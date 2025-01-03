#include <stdio.h>
#include <stdlib.h> // 用于rand和srand函数
#include <unistd.h> // 用于usleep函数
#include <time.h>   // 用于时间函数
#include <limits.h> // 用于INT_MAX

#define NUM_DRONES_PER_CLUSTER 10
#define NUM_CLUSTERS 10
#define TDMA_SLOT_TIME_US 50000 // 每个无人机的时间槽，以微秒为单位（0.05秒）
#define TOTAL_TIME_SLOTS 60     // 总模拟时隙数（调整以匹配新的时隙长度）

// 数据包大小（字节）
#define PACKET_SIZE 256

// 定义表示无人机的Node结构
typedef struct {
    int id;           // 无人机ID
    double send_will; // 发送欲望
    int is_head;      // 是否为簇头
    double x, y;      // 无人机的位置坐标
    int energy;       // 节点的能量
} Node;

// 定义表示信道的Channel结构
typedef struct {
    int state; // 信道状态：0为空闲，1为繁忙
} Channel;

// 定义表示簇的Cluster结构
typedef struct {
    int id;
    Node drones[NUM_DRONES_PER_CLUSTER];
    Channel channel; // 每个簇都有一个信道
    int head_id;     // 簇头节点的ID
} Cluster;

// 全局变量，用于统计每个簇的发送次数、延迟时间和总传输字节数
volatile int counts[NUM_CLUSTERS] = {0};
volatile double delaytimes[NUM_CLUSTERS] = {0.0};
volatile int total_transmissions[NUM_CLUSTERS] = {0};
volatile double total_delay_time[NUM_CLUSTERS] = {0.0};
volatile long total_bytes_transmitted[NUM_CLUSTERS] = {0}; // 新增：统计传输的总字节数

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
        }
        // 设置簇头
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            if (clusters[c].drones[d].id == min_id) {
                clusters[c].drones[d].is_head = 1;
                clusters[c].head_id = min_id;
                break;
            }
        }
        clusters[c].channel.state = 0; // 初始化信道为空闲状态
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
void generate_send_will(Cluster clusters[]) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].send_will = (double)rand() / RAND_MAX;
        }
        // 对当前簇内的无人机按发送欲望排序
        sort_drones_by_send_will(clusters[c].drones, NUM_DRONES_PER_CLUSTER);
    }
}

// 模拟无人机发送数据
void send_data(Node* drone, Cluster* cluster) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    printf("Drone %d at (%.2f, %.2f) in Cluster %d", drone->id, drone->x, drone->y, cluster->id);
    if (drone->is_head) {
        printf(" (Head)");
    }
    printf(" is sending data.\n");

    counts[cluster->id]++;
    
    // 设置信道为繁忙状态
    cluster->channel.state = 1;

    // 模拟一些处理延迟（示例目的）
    usleep(5000); // 睡眠5毫秒以模拟发送延迟

    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    delaytimes[cluster->id] += elapsed_time;

    // 更新整个模拟过程中的总传输次数和总延迟时间
    total_transmissions[cluster->id]++;
    total_delay_time[cluster->id] += elapsed_time;
    total_bytes_transmitted[cluster->id] += PACKET_SIZE; // 更新传输的总字节数

    // 减少节点的能量
    drone->energy--;

    // 设置信道为空闲状态
    cluster->channel.state = 0;
}

// 每一轮或模拟结束时打印统计数据
void print_statistics(int round_counter) {
    if (round_counter > 0) {

        printf("--------------------------------------\n");
        printf("Round %d:\n", round_counter);
        for (int c = 0; c < NUM_CLUSTERS; ++c) {
            double avg_delay = counts[c] > 0 ? delaytimes[c] / counts[c] : 0;
            double total_delay = counts[c] * avg_delay;
            printf("Cluster %d: Total transmissions: %d, Average delay time: %.6f seconds, Total delay time: %.6f seconds\n",
                   c + 1, counts[c], avg_delay, total_delay);
        }
        printf("--------------------------------------\n");    
    }
}

// 模拟结束后输出最终统计数据
void print_final_statistics() {
    printf("\n");
    printf("\n");
    printf("Simulation ended after %d time slots.\n", TOTAL_TIME_SLOTS);
    printf("--------------------------------------\n"); 
    printf("Final statistics after the entire simulation:\n");
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        double throughput = total_delay_time[c] > 0 ? (total_bytes_transmitted[c] / total_delay_time[c]) : 0;
        printf("Cluster %d: Total packets sent: %d, Total delay time: %.6f seconds, Total bytes transmitted: %ld, Throughput: %.6f B/s\n",
               c + 1, total_transmissions[c], total_delay_time[c], total_bytes_transmitted[c], throughput);
    }
    printf("--------------------------------------\n"); 
}

void update_cluster(Cluster* cluster, int current_time){
    Node* drone = &cluster->drones[current_time%NUM_DRONES_PER_CLUSTER];
    if (cluster->channel.state == 0 && drone->energy > 0) {
        send_data(drone, cluster);
    } else if (cluster->channel.state != 0) {
        printf("Channel of Cluster %d is currently busy, skipping this transmission.\n", cluster->id);
    } else {
        printf("Skipping transmission from Drone %d in Cluster %d due to lack of energy.\n", drone->id, cluster->id);
    }
}

void show_slot_start(int slot_counter){
    printf("--------------------------------------\n");
    printf("Now is %d slot:\n", slot_counter);
}

void show_slot_stop(int slot_counter){
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
        if (slot_counter % 10 == 0) generate_send_will(clusters);

        // 更新当前时隙下的每个簇
        for (int c = 0; c < NUM_CLUSTERS; ++c) {
            update_cluster(&clusters[c], slot_counter);
        }

        // 每个时间槽之间等待一段时间
        usleep(TDMA_SLOT_TIME_US);
        slot_counter++;
        show_slot_stop(slot_counter);   

        if (slot_counter % 10 == 0) {
            round_counter++;
            print_statistics(round_counter);
            // 在每轮结束时重置每个簇的统计信息
            for (int c = 0; c < NUM_CLUSTERS; ++c) {
                counts[c] = 0;
                delaytimes[c] = 0.0;
            }
        }

        // 如果达到了总时隙数，结束模拟
        if (slot_counter >= TOTAL_TIME_SLOTS) break;
    }

    // 模拟结束后输出最终统计数据
    print_final_statistics();
}

int main() {
    srand(time(NULL)); // 初始化随机数种子

    Cluster clusters[NUM_CLUSTERS];
    initialize_clusters(clusters);

    // 开始模拟
    simulate_tdma_communication(clusters);

    // 打印所有节点剩余能量
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            printf("Drone %d in Cluster %d has remaining energy: %d\n", clusters[c].drones[d].id, c + 1, clusters[c].drones[d].energy);
        }
    }

    return 0;
}