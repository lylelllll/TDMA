#include <stdio.h>
#include <stdlib.h> // 用于rand和srand函数
#include <time.h>   // 用于时间函数
#include <limits.h> // 用于INT_MAX

#define NUM_DRONES_PER_CLUSTER 20
#define NUM_CLUSTERS 15
#define SLOT_TIME 51.2 // 每个时隙的时间长度，以微秒为单位
#define TOTAL_TIME_SLOTS 140 // 总模拟时隙数（调整以匹配新的时隙长度）

// 数据包大小（字节）
#define PACKET_SIZE 256


//退避参数
#define R1 0.3
#define R2 0.7
#define CW_P1 8
#define CW_P2 16
#define CW_P3 24

// 定义一个枚举类型来表示不同的信道状态
typedef enum {
    CHANNEL_CLASH = -1,   // 信道冲突
    CHANNEL_IDLE = 0,     // 信道空闲
    CHANNEL_RTS = 1,      // 被请求帧占有
    CHANNEL_CTS = 2,      // 被响应帧占有
    CHANNEL_DATA = 3,     // 被数据包占有
    CHANNEL_EXTRA = 4,    // 被簇间通信占有
    CHANNEL_ACI = 5,      // 被确认帧占有
    CHANNEL_BEACON = 6,   // 被信标占有
    CHANNEL_PACKET = 7    // 被包占有
} ChannelState;

// 定义表示无人机的Node结构
typedef struct {
    int id;           // 无人机ID
    double send_will; // 发送欲望
    int is_head;      // 是否为簇头
    double x, y;      // 无人机的位置坐标
    int energy;       // 节点的能量
    int start_slot;   // 发送开始的时隙编号
    int end_slot;     // 发送成功的时隙编号
    bool want_to_send; //节点是否有数据要发
    int back_off_slot; //节点要退避的时隙数
    bool is_dead;
    int dead_slot;
    bool delay_first;
    int total_delay_slot;
} Node;

// 定义表示信道的Channel结构
typedef struct {
    ChannelState state; // 信道状态
    int state_update_slot;
} Channel;

// 定义表示簇的Cluster结构
typedef struct {
    int id;
    Node drones[NUM_DRONES_PER_CLUSTER];
    Channel channel; // 每个簇都有一个信道
    int head_id;     // 簇头节点的ID
    int node_num; //簇内节点数量
} Cluster;

// 全局变量，用于统计每个簇的发送次数、延迟时间和总传输字节数
volatile int total_transmissions[NUM_CLUSTERS][NUM_DRONES_PER_CLUSTER] = {0};
volatile double total_delay_time[NUM_CLUSTERS][NUM_DRONES_PER_CLUSTER] = {0.0};
volatile int total_clash_slot[NUM_CLUSTERS] = {0};
volatile int total_idle_slot[NUM_CLUSTERS] = {0};


// 初始化簇并分配无人机ID，并确定簇头和随机位置坐标
void initialize_clusters(Cluster clusters[]) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        clusters[c].id = c;
        clusters[c].node_num = NUM_DRONES_PER_CLUSTER;

        double range_start = c * 1000; // 根据簇ID确定坐标范围起点
        double range_end = (c + 1) * 1000; // 根据簇ID确定坐标范围终点

        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].id = c * NUM_DRONES_PER_CLUSTER + d + 1;
            clusters[c].drones[d].is_head = 0;
            // 为无人机分配随机位置坐标，基于当前簇的坐标范围
            clusters[c].drones[d].x = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;
            clusters[c].drones[d].y = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;
            // 为无人机分配初始能量（随机5-10之间的整数）
            clusters[c].drones[d].energy = rand() % 6 + 5; // 随机整数范围 [5, 10]
            clusters[c].drones[d].start_slot = -1; // 初始化发送开始时隙编号
            clusters[c].drones[d].end_slot = -1;   // 初始化发送成功时隙编号
            clusters[c].drones[d].want_to_send = false;
            clusters[c].drones[d].is_dead = false;
            clusters[c].drones[d].delay_first = true;
            clusters[c].drones[d].back_off_slot = 0;

        }
        // 设置簇头
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            if (clusters[c].drones[d].id == min_id) {
                clusters[c].drones[d].is_head = 1;
                clusters[c].head_id = min_id;
                break;
            }
        }
        clusters[c].channel.state = CHANNEL_IDLE; // 初始化信道为空闲状态
    }
}



// 模拟无人机想发送数据
void random_want_to_send(Cluster clusters[], int current_slot) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            if((double)rand() / RAND_MAX < 0.5){
                if(clusters[c].drones[d].want_to_send==false){
                    clusters[c].drones[d].want_to_send=true;
                    clusters[c].drones[d].back_off_slot=0;

                }
            }
        }
    }
}




// 模拟无人机发送数据
void send_data(Node* drone, Cluster* cluster, int slot_counter) {
    printf("Drone %d at (%.2f, %.2f) in Cluster %d", drone->id, drone->x, drone->y, cluster->id);
    if (drone->is_head) {
        printf(" (Head)");
    }
    
    // 设置信道为被数据包占有状态
    cluster->channel.state = CHANNEL_DATA;

    // 更新整个模拟过程中的总传输次数和总延迟时间
    total_transmissions[cluster->id%NUM_CLUSTERS][drone->id%NUM_DRONES_PER_CLUSTER]++;

    // 减少节点的能量
    drone->energy--;

    drone->end_slot = slot_counter; // 记录发送成功时隙编号
    double delaytime = (drone->end_slot-drone->start_slot+1)*SLOT_TIME;
    printf(" sent finish. Delay_time:%.6f ms\n",delaytime/1000);
    total_delay_time[cluster->id%NUM_CLUSTERS][drone->id%NUM_DRONES_PER_CLUSTER] += delaytime;

    // 设置信道为空闲状态
    cluster->channel.state = CHANNEL_IDLE;
}


// 模拟结束后输出最终统计数据
void print_final_statistics(Cluster clusters[]) {
    printf("\n");
    printf("\n");
    printf("Simulation ended after %d time slots.\n", TOTAL_TIME_SLOTS);
    printf("--------------------------------------\n"); 
    printf("Final statistics after the entire simulation:\n");
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            Node drone = clusters[c].drones[d];
            double avg_delaytime = total_delay_time[c][d]/total_transmissions[c][d]/1000;
            double avg_throughput = ((total_transmissions[c][d]*PACKET_SIZE)/total_delay_time[c][d])*1000000/(1024*1024);
            printf("Drone %d in Cluster %d avg_delaytime: %.6fms avg_throughput: %.6fMB/s  remaining energy: %d\n", drone.id,  clusters[c].id, avg_delaytime, avg_throughput,clusters[c].drones[d].energy);


        }

    }
    printf("--------------------------------------\n"); 
}

int judge_clash(Cluster* cluster, int current_slot){
    int count = 0;
    for(int i=0;i<cluster->node_num;i++){
        if(cluster[i]->want_to_send)count++;
    }
    return count;
}

void back_off(Cluster* cluster, int current_slot){
    for (int j = 0; j < cluster->node_num; ++j) {
            if (cluster->drones[j].want_to_send && cluster->drones[j].id != cluster->head_id &&
                cluster->drones[j].energy > 0 && cluster->drones[j].back_off_slot == 0) {
                double RE_W = (double)cluster->drones[j].energy / 72;
                int DP = '01'; 
                int CW_DP = 0;

                if (DP == '00') {
                    CW_DP = CW_P1;
                } else if (DP == '01') {
                    CW_DP = CW_P2;
                } else {
                    CW_DP = CW_P3;
                }

                int ZREi_w = 0;
                if (0 <= RE_W && RE_W < R1) {
                    ZREi_w = 3;
                } else if (R1 <= RE_W && RE_W < R2) {
                    ZREi_w = 2;
                } else {
                    ZREi_w = 1;
                }

                int tuibi_time = rand() % (CW_DP / pow(2, ZREi_w)) + 1;
                cluster->drones[j].back_off_slot = tuibi_time * 8;
            }
        }

}

void update_drone(Cluster* cluster,Node* node, int current_slot){
    send_rts();
    send_cts();
    send_data();
    send_aci();
    send_beacon();
    send_packet();
    drone_sleep();

    //判断能量
    if(node->energy<1){
        node->is_dead = true;
        node->dead_slot = current_slot;
    }
    //更新退避时隙
    if(node->back_off_slot > 0) node->back_off_slot--;

    if(node->want_to_send){
        if(node->delay_first){
            node->start_slot = current_slot;
            node->delay_first = false;
        }
        
        if(node->success_flag){
            if(current_slot-node->start_slot>=8){
                node->total_delay_slot += current_slot-node->start_slot;
            }
            node->want_to_send = false;
            node->delay_first = true;
        }
    }


}

void update_cluster(Cluster* cluster, int current_slot){
    if (cluster->channel.state == CHANNEL_IDLE || cluster->channel.state == CHANNEL_CLASH)
    {
        //统计当前时隙下想要发数据的节点个数
        int clash_nums = judge_clash(cluster,current_slot);


        if(clash_nums>1){
            //发生冲突
            cluster->channel.state = CHANNEL_CLASH;
            total_clash_slot[cluster->id]++;
            back_off(cluster, current_slot);
        }else if (clash_nums==1){
            cluster->channel.state = CHANNEL_RTS;

        }else{
            cluster->channel.state = CHANNEL_IDLE;
            total_idle_slot[cluster->id]++;
        }
        cluster->channel.state_update_slot=current_slot;
    }
    //逐个节点进行更新
    for (int i = 0; i < cluster.node_num; ++i)
    {
        Node* drone = &cluster->drones[i];
        update_drone(cluster,drone,current_slot);
    }

    update_channel(cluster,current_slot);



}

void show_slot_start(int slot_counter){
    printf("--------------------------------------\n");
    printf("Now is %d slot:\n", slot_counter);
}

void show_slot_stop(){
    printf("--------------------------------------\n");
    printf("\n");
    printf("\n");
}

// 处理整个TDMA通信模拟过程
void simulate_tdma_communication(Cluster clusters[]) {
    int slot_counter = 0; // 跟踪时隙的计数器
    int round_counter = 0; // 跟踪轮次的计数器

    while (slot_counter < TOTAL_TIME_SLOTS) { // 模拟循环
        show_slot_start(slot_counter);

        // 每过一段时间随机模拟无人机想发数据
        if (slot_counter % 20 == 0) random_want_to_send(clusters,slot_counter);

        // 更新当前时隙下的每个簇
        for (int c = 0; c < NUM_CLUSTERS; ++c) {
            update_cluster(&clusters[c], slot_counter);
        }

        slot_counter++;
        show_slot_stop();   

        if (slot_counter % 10 == 0) {
            round_counter++;
        }

        // 如果达到了总时隙数，结束模拟
        if (slot_counter >= TOTAL_TIME_SLOTS) break;
    }

    // 模拟结束后输出最终统计数据
    print_final_statistics(clusters);

}

int main() {
    srand(time(NULL)); // 初始化随机数种子

    Cluster clusters[NUM_CLUSTERS];
    initialize_clusters(clusters);

    // 开始模拟
    simulate_tdma_communication(clusters);


    return 0;
}