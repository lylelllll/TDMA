#include <stdio.h>
#include <stdlib.h> // 用于rand和srand函数
#include <time.h>   // 用于时间函数
#include <limits.h> // 用于INT_MAX

#define NUM_DRONES_PER_CLUSTER 10
#define NUM_CLUSTERS 10
#define SLOT_TIME 51.2 // 每个时隙的时间长度，以微秒为单位
#define TOTAL_TIME_SLOTS 60 // 总模拟时隙数（调整以匹配新的时隙长度）

// 数据包大小（字节）
#define PACKET_SIZE 256

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
} Node;

// 定义表示信道的Channel结构
typedef struct {
    ChannelState state; // 信道状态
} Channel;

// 定义表示簇的Cluster结构
typedef struct {
    int id;
    Node drones[NUM_DRONES_PER_CLUSTER];
    Channel channel; // 每个簇都有一个信道
    int head_id;     // 簇头节点的ID
} Cluster;

// 全局变量，用于统计每个簇的发送次数、延迟时间和总传输字节数
volatile int total_transmissions[NUM_CLUSTERS][NUM_DRONES_PER_CLUSTER] = {0};
volatile double total_delay_time[NUM_CLUSTERS][NUM_DRONES_PER_CLUSTER] = {0.0};


// 初始化簇并分配无人机ID，并确定簇头和随机位置坐标
void initialize_clusters(Cluster clusters[]) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        clusters[c].id = c;
        int min_id = INT_MAX;
        double range_start = c * 1000; // 根据簇ID确定坐标范围起点
        double range_end = (c + 1) * 1000; // 根据簇ID确定坐标范围终点

        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].id = c * NUM_DRONES_PER_CLUSTER + d + 1;
            clusters[c].drones[d].is_head = 0;
            if (clusters[c].drones[d].id < min_id) {
                min_id = clusters[c].drones[d].id;
            }
            // 为无人机分配随机位置坐标，基于当前簇的坐标范围
            clusters[c].drones[d].x = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;
            clusters[c].drones[d].y = ((double)rand() / RAND_MAX) * (range_end - range_start) + range_start;
            // 为无人机分配初始能量（随机5-10之间的整数）
            clusters[c].drones[d].energy = rand() % 6 + 5; // 随机整数范围 [5, 10]
            clusters[c].drones[d].start_slot = -1; // 初始化发送开始时隙编号
            clusters[c].drones[d].end_slot = -1;   // 初始化发送成功时隙编号
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

// 快速排序的分区函数
int partition(Node arr[], int low, int high) {
    double pivot = arr[high].send_will;
    int i = low - 1;

    for (int j = low; j <= high - 1; j++) {
        if (arr[j].send_will >= pivot) {
            i++;
            // 交换两个无人机的位置
            Node temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    // 交换pivot元素
    Node temp = arr[i + 1];
    arr[i + 1] = arr[high];
    arr[high] = temp;
    return i + 1;
}

// 快速排序函数
void quick_sort(Node arr[], int low, int high) {
    if (low < high) {
        int pi = partition(arr, low, high);

        // 递归调用快速排序
        quick_sort(arr, low, pi - 1);
        quick_sort(arr, pi + 1, high);
    }
}

// 根据发送欲望对无人机进行排序
void sort_drones_by_send_will(Node drones[], int num_drones) {
    quick_sort(drones, 0, num_drones - 1);
}

// 生成每个无人机的发送欲望
void generate_send_will(Cluster clusters[], int current_slot) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].send_will = (double)rand() / RAND_MAX;
            clusters[c].drones[d].start_slot = current_slot;
        }
        // 对当前簇内的无人机按发送欲望排序
        sort_drones_by_send_will(clusters[c].drones, NUM_DRONES_PER_CLUSTER);
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
    total_transmissions[cluster->id%10][drone->id%10]++;

    // 减少节点的能量
    drone->energy--;

    drone->end_slot = slot_counter; // 记录发送成功时隙编号
    double delaytime = (drone->end_slot-drone->start_slot+1)*SLOT_TIME;
    printf(" sent finish. Delay_time:%.6f ms\n",delaytime);
    total_delay_time[cluster->id%10][drone->id%10] += delaytime;

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
            double avg_delaytime = total_delay_time[c][d]/total_transmissions[c][d];
            double avg_throughput = ((total_transmissions[c][d]*PACKET_SIZE)/total_delay_time[c][d])*1000000/1024;
            printf("Drone %d in Cluster %d avg_delaytime: %.6fms avg_throughput: %.6fKB/s  remaining energy: %d\n", drone.id,  clusters[c].id, avg_delaytime, avg_throughput,clusters[c].drones[d].energy);


        }

    }
    printf("--------------------------------------\n"); 
}

void update_cluster(Cluster* cluster, int current_time){
    Node* drone = &cluster->drones[current_time%NUM_DRONES_PER_CLUSTER];
    if (cluster->channel.state == CHANNEL_IDLE && drone->energy > 0) {
        send_data(drone, cluster, current_time);
    } else if (cluster->channel.state != CHANNEL_IDLE) {
        printf("Channel of Cluster %d is currently busy with state %d, skipping this transmission.\n", cluster->id, cluster->channel.state);
    } else {
        printf("Skipping transmission from Drone %d in Cluster %d due to lack of energy.\n", drone->id, cluster->id);
    }
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

        // 每一轮开始时生成发送欲望并排序
        if (slot_counter % 10 == 0) generate_send_will(clusters,slot_counter);

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