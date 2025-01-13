#include <stdio.h>
#include <stdlib.h> // 用于rand和srand函数
#include <time.h>   // 用于时间函数
#include <limits.h> // 用于INT_MAX
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define NUM_DRONES_PER_CLUSTER 20
#define NUM_CLUSTERS 1
#define SLOT_TIME 51.2 // 每个时隙的时间长度，以微秒为单位
#define TOTAL_TIME_SLOTS 1000 // 总模拟时隙数（调整以匹配新的时隙长度）


//退避参数
#define R1 0.3
#define R2 0.7
#define CW_P1 8
#define CW_P2 16
#define CW_P3 24

// 请求帧大小 (单位: bits)
#define RTS_SIZE (15 * 8)
// 请求帧时隙
#define RTS_SLOT 1

// 响应帧大小 (单位: bits)
#define CTS_SIZE (15 * 8)
// 响应帧时隙
#define CTS_SLOT 1

// 确认帧大小 (单位: bits)
#define ACI_SIZE (40 * 8)
// 确认帧时隙
#define ACI_SLOT 1

// 信标大小 (单位: bits)
#define BEACON_SIZE (8 * 8)
// 信标时隙
#define BEACON_SLOT 1

// 数据包大小 (单位: bits)
#define DATA_SIZE (20 * 8)
// 数据包时隙
#define DATA_SLOT 1

// 大数据包大小 (单位: bits)
#define PACKET_SIZE (200 * 8)
// 大数据包时隙
#define PACKET_SLOT 3


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
    int is_head;      // 是否为簇头
    double x, y;      // 无人机的位置坐标
    int energy;       // 节点的能量
    int start_slot;   // 发送开始的时隙编号
    int end_slot;     // 发送成功的时隙编号
    bool want_to_send; //节点是否有数据要发
    bool able_send; //节点能否发
    int back_off_slot; //节点要退避的时隙数
    bool is_dead;
    int dead_slot;
    bool delay_first;
    int total_delay_slot;
    int total_sent_packet;
    int total_throught_put;
    bool success_flag;
} Node;

// 定义表示信道的Channel结构
typedef struct {
    ChannelState state; // 信道状态
    int state_update_slot;
    int owner_id;
    int nch_id;
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
volatile int total_clash_slot[NUM_CLUSTERS] = {0};
volatile int total_idle_slot[NUM_CLUSTERS] = {0};
volatile int total_rts[NUM_CLUSTERS] = {0};
volatile int total_cts[NUM_CLUSTERS] = {0};
volatile int total_data[NUM_CLUSTERS] = {0};
volatile int total_aci[NUM_CLUSTERS] = {0};
volatile int total_beacon[NUM_CLUSTERS] = {0};
volatile int total_packet[NUM_CLUSTERS] = {0};




// 初始化簇并分配无人机ID，并确定簇头和随机位置坐标
void initialize_clusters(Cluster clusters[]) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        clusters[c].id = c;
        clusters[c].node_num = NUM_DRONES_PER_CLUSTER;

        double range_start = c * 1000; // 根据簇ID确定坐标范围起点
        double range_end = (c + 1) * 1000; // 根据簇ID确定坐标范围终点

        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].id = c * NUM_DRONES_PER_CLUSTER + d + 1;

            // 为无人机分配初始能量（随机5-10之间的整数）
            clusters[c].drones[d].energy = rand() % 23 + 50; // 随机整数范围 [50, 72]

            if(d)clusters[c].drones[d].is_head = 0;
            else {//0号节点为簇头
                clusters[c].drones[d].is_head =1;
                clusters[c].drones[d].energy = 10000;//簇头节点能量大
            }

            // 为无人机分配随机位置坐标，基于当前簇的坐标范围
            clusters[c].drones[d].x = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;
            clusters[c].drones[d].y = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;



            clusters[c].drones[d].start_slot = -1; // 初始化发送开始时隙编号
            clusters[c].drones[d].end_slot = -1;   // 初始化发送成功时隙编号
            clusters[c].drones[d].want_to_send = false;
            clusters[c].drones[d].able_send = false;
            clusters[c].drones[d].is_dead = false;
            clusters[c].drones[d].delay_first = true;
            clusters[c].drones[d].success_flag = false;

            clusters[c].drones[d].back_off_slot = 0;
            clusters[c].drones[d].total_delay_slot = 0;
            clusters[c].drones[d].total_sent_packet = 0;
            clusters[c].drones[d].dead_slot = -1;

        }


        //簇内信道
        clusters[c].channel.state = CHANNEL_IDLE; // 初始化信道为空闲状态
        clusters[c].channel.owner_id = -1;
        clusters[c].channel.nch_id = -1;


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







// 模拟结束后输出最终统计数据
void print_final_statistics(Cluster clusters[]) {
    printf("\n");
    printf("\n");
    printf("Simulation ended after %d time slots.\n", TOTAL_TIME_SLOTS);
    printf("--------------------------------------\n"); 
    printf("Final statistics after the entire simulation:\n");
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        printf("(Cluster%d's intra Channel) total_packet: %d, total_idle_slot: %d, total_clash_slot: %d\n", clusters[c].id, total_packet[clusters[c].id], total_idle_slot[clusters[c].id], total_clash_slot[clusters[c].id]);
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            Node drone = clusters[c].drones[d];
            if(drone.total_delay_slot&&drone.total_sent_packet){
                double avg_delaytime = (drone.total_delay_slot*SLOT_TIME)/drone.total_sent_packet;
                double avg_throughput = (drone.total_sent_packet*PACKET_SIZE)/(drone.total_delay_slot*SLOT_TIME);
                printf("Drone %d in Cluster %d have sent: %d packets,  avg_delaytime: %.6fms avg_throughput: %.6fb/ms  remaining energy: %d\n", drone.id,  clusters[c].id, drone.total_sent_packet, avg_delaytime/1000, avg_throughput*1000,drone.energy);
            }else{
                printf("Drone %d in Cluster %d haven't sent\n",drone.id, clusters[c].id);
            }


        }

    }
    printf("--------------------------------------\n"); 
}

bool judge_energy(Node* node){
    return node->energy > 1;
}

bool judge_send(Cluster* cluster,Node* node, int current_slot){

    if(node->want_to_send==false){
        node->able_send = false;
        return false;
    }

    if(!judge_energy(node))return false;

    if(node->is_head)node->able_send = false;

    //信道为响应和数据时，必须占有才可以发
    if(cluster->channel.state == CHANNEL_DATA || cluster->channel.state == CHANNEL_CTS){
        if(cluster->channel.owner_id != node->id){
            node->able_send = false;
            return false;
        }

    }

    if(node->back_off_slot != 0){
        node->able_send = false;
        return false;
    }

    node->able_send = true;
    return true;

}

int judge_clash(Cluster* cluster, int current_slot){
    int count = 0;
    for(int i=0;i<cluster->node_num;i++){
        if(judge_send(cluster,&cluster->drones[i],current_slot))count++;
    }
    return count;
}

void back_off(Cluster* cluster, int current_slot){
    for (int j = 0; j < cluster->node_num; ++j) {
            if (cluster->drones[j].want_to_send && cluster->drones[j].id != cluster->head_id &&
                cluster->drones[j].energy > 0 && cluster->drones[j].back_off_slot == 0) {
                double RE_W = (double)cluster->drones[j].energy / 72;
                char* DP = "01"; 
                int CW_DP = 0;

                if (strcmp(DP, "00") == 0) {
                    CW_DP = CW_P1;
                } else if (strcmp(DP, "01") == 0) {
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

                int tuibi_time = rand() % ((int)(CW_DP / pow(2, ZREi_w))) + 1;
                cluster->drones[j].back_off_slot = tuibi_time * 8;
                printf("drone %d back_off_slot %d\n",cluster->drones[j].id,cluster->drones[j].back_off_slot);
            }
        }

}




void send_rts(Cluster* cluster,Node* node, int current_slot){

    //簇头不发rts
    if(node->is_head)return;

    //clash状态或rts状态才发rts
    if(cluster->channel.state != CHANNEL_RTS && cluster->channel.state != CHANNEL_CLASH)return;

    //判断能量
    if(!judge_energy(node))return;

    //判断能不能发
    if(node->able_send && cluster->channel.state != CHANNEL_CLASH){
        node->energy-=1;
        node->able_send = false;

        printf("Drone %d at (%.2f, %.2f) in Cluster %d send rts\n", node->id, node->x, node->y, cluster->id);

        cluster->channel.owner_id = node->id;

    }
    //判断是否发送成功
    if(RTS_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_RTS && cluster->channel.owner_id == node->id){
        printf("Drone %d in Cluster %d successfully send rts\n", node->id, cluster->id);
        total_rts[cluster->id]+=1;

        cluster->channel.owner_id = node->id;
        cluster->channel.nch_id = node->id;

    }

}

void send_cts(Cluster* cluster,Node* node, int current_slot){
    //是簇头才发cts
    if(!node->is_head)return;

    if(!judge_energy(node))return;

    if(cluster->channel.state == CHANNEL_CTS){
        node->energy -= 1;
        printf("Cluster Head %d at (%.2f, %.2f) in Cluster %d send cts\n", node->id, node->x, node->y, cluster->id);

        cluster->channel.owner_id = node->id;
    }
    else{
        return;
    }

    //判断是否发送成功
    if(CTS_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_CTS){
        printf("Cluster Head %d in Cluster %d successfully send cts\n", node->id, cluster->id);

        total_cts[cluster->id]+=1;

        cluster->channel.owner_id = node->id;

    }

}

void send_data(Cluster* cluster,Node* node, int current_slot){
    //簇头不发data
    if(node->is_head)return;

    //判断能量
    if(!judge_energy(node))return;

    //判断能不能发
    if(cluster->channel.nch_id == node->id && cluster->channel.state == CHANNEL_DATA){
        node->energy-=1;

        printf("Drone %d at (%.2f, %.2f) in Cluster %d send data\n", node->id, node->x, node->y, cluster->id);

    }else{
        return;
    }
    //判断是否发送成功
    if(DATA_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_DATA){
        printf("Drone %d in Cluster %d successfully send data\n", node->id, cluster->id);
        total_data[cluster->id]+=1;

    }

    
}

void send_aci(Cluster* cluster,Node* node, int current_slot){

    //虽然是簇头才发aci，但模拟的过程也可以由接收方模拟发?
    if(!node->is_head)return;

    if(!judge_energy(node))return;

    if(cluster->channel.state == CHANNEL_ACI){//?nch_id==drone->id
        node->energy -= 1;
        printf("Cluster Head %d at (%.2f, %.2f) in Cluster %d send aci\n", node->id, node->x, node->y, cluster->id);

    }
    else{
        return;
    }

    //判断是否发送成功
    if(ACI_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_ACI){
        printf("Cluster Head %d in Cluster %d successfully send aci\n", node->id, cluster->id);

        total_aci[cluster->id]+=1;

        cluster->channel.owner_id = node->id;

    }

    
}

void send_beacon(Cluster* cluster,Node* node, int current_slot){
    //簇头才发beacon
    if(!node->is_head)return;

    if(!judge_energy(node))return;

    if(cluster->channel.state == CHANNEL_BEACON){//?nch_id==drone->id
        node->energy -= 1;
        printf("Cluster Head %d at (%.2f, %.2f) in Cluster %d send beacon\n", node->id, node->x, node->y, cluster->id);

    }
    else{
        return;
    }

    //判断是否发送成功
    if(BEACON_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_BEACON){
        printf("Cluster Head %d in Cluster %d successfully send beacon\n", node->id, cluster->id);

        total_beacon[cluster->id]+=1;

        cluster->channel.owner_id = node->id;

    }
    
}

void send_packet(Cluster* cluster,Node* node, int current_slot){
    //簇头不发packet
    if(node->is_head)return;

    //判断能量
    if(!judge_energy(node))return;

    //判断能不能发
    if(cluster->channel.nch_id == node->id && cluster->channel.state == CHANNEL_PACKET){
        node->energy-=1;

        printf("Drone %d at (%.2f, %.2f) in Cluster %d send packet\n", node->id, node->x, node->y, cluster->id);

    }else{
        return;
    }
    //判断是否发送成功
    if(PACKET_SLOT == current_slot - cluster->channel.state_update_slot && cluster->channel.state == CHANNEL_PACKET){
        printf("Drone %d in Cluster %d successfully send packet\n", node->id, cluster->id);
        total_packet[cluster->id]+=1;
        node->success_flag = true;

    }
    
}

//没用
void drone_sleep(Cluster* cluster,Node* node, int current_slot){
    if(node->is_head)return;
    if(node->back_off_slot>0)return;
    
}

void update_drone(Cluster* cluster,Node* node, int current_slot){
    send_rts(cluster,node,current_slot);
    send_cts(cluster,node,current_slot);
    send_data(cluster,node,current_slot);
    send_aci(cluster,node,current_slot);
    send_beacon(cluster,node,current_slot);
    send_packet(cluster,node,current_slot);
    drone_sleep(cluster,node,current_slot);

    //判断能量
    if(!judge_energy(node)){
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
                node->total_sent_packet += 1;
                node->total_throught_put += PACKET_SIZE;
            }
            node->want_to_send = false;
            node->delay_first = true;
            node->able_send = false;
            node->success_flag = false;
        }
    }


}

void update_channel(Cluster* cluster, int current_slot){
    Channel* channel = &cluster->channel;
    printf("channel state: %d\n",channel->state);


    if(channel->state == CHANNEL_RTS && RTS_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_CTS;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_CTS && CTS_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_DATA;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_DATA && DATA_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_ACI;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_ACI && ACI_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_BEACON;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_BEACON && BEACON_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_PACKET;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_PACKET && PACKET_SLOT == current_slot - channel->state_update_slot){
        channel->state = CHANNEL_IDLE;
        channel->state_update_slot = current_slot;
    }

    if(channel->state == CHANNEL_IDLE || channel->state == CHANNEL_CLASH){
        channel->state_update_slot = current_slot;
    }


}

void update_cluster(Cluster* cluster, int current_slot){
    if (cluster->channel.state == CHANNEL_IDLE || cluster->channel.state == CHANNEL_CLASH)
    {
        //统计当前时隙下想要发数据的节点个数
        int clash_nums = judge_clash(cluster,current_slot);


        if(clash_nums>1){
            //发生冲突
            printf("Cluster %d clash number: %d\n", cluster->id , clash_nums);

            cluster->channel.state = CHANNEL_CLASH;
            total_clash_slot[cluster->id]++;
            back_off(cluster, current_slot);
        }else if (clash_nums==1){
            cluster->channel.state = CHANNEL_RTS;

        }else{
            printf("Cluster %d idle\n", cluster->id);
            cluster->channel.state = CHANNEL_IDLE;
            total_idle_slot[cluster->id]++;
        }
        
    }
    //逐个节点进行更新
    for (int i = 0; i < cluster->node_num; ++i)
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
        if (slot_counter % 10 == 0) random_want_to_send(clusters,slot_counter);

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