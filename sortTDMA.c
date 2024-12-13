#include <stdio.h>
#include <stdlib.h> // 用于rand和srand函数
#include <unistd.h> // 用于usleep函数
#include <time.h>   // 用于时间函数
#include <limits.h>

#define NUM_DRONES_PER_CLUSTER 10
#define NUM_CLUSTERS 10
#define TDMA_SLOT_TIME_US 50000 // 每个无人机的时间槽，以微秒为单位（0.05秒）
#define TOTAL_TIME_SLOTS 2000   // 总模拟时隙数（调整以匹配新的时隙长度）

// 数据包大小（字节）
#define PACKET_SIZE 256

// 定义表示无人机的Node结构
typedef struct {
    int id;           // 无人机ID
    double send_will; // 发送欲望
    int is_head;      // 是否为簇头
} Node;

// 定义表示信道的Channel结构
typedef struct {
    int state; // 信道状态：0为空闲，1为繁忙
} Channel;

// 定义表示簇的Cluster结构
typedef struct {
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

// 初始化簇并分配无人机ID，并确定簇头
void initialize_clusters(Cluster clusters[]) {
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        int min_id = INT_MAX;
        for (int d = 0; d < NUM_DRONES_PER_CLUSTER; ++d) {
            clusters[c].drones[d].id = c * NUM_DRONES_PER_CLUSTER + d + 1;
            clusters[c].drones[d].is_head = 0;
            if (clusters[c].drones[d].id < min_id) {
                min_id = clusters[c].drones[d].id;
            }
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
void send_data(Node* drone, int cluster_id, Cluster clusters[]) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    printf("Drone %d in Cluster %d", drone->id, cluster_id + 1);
    if (drone->is_head) {
        printf(" (Head)");
    }
    printf(" is sending data.\n");

    counts[cluster_id]++;
    
    // 设置信道为繁忙状态
    clusters[cluster_id].channel.state = 1;

    // 模拟一些处理延迟（示例目的）
    usleep(5000); // 睡眠5毫秒以模拟发送延迟

    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    delaytimes[cluster_id] += elapsed_time;

    // 更新整个模拟过程中的总传输次数和总延迟时间
    total_transmissions[cluster_id]++;
    total_delay_time[cluster_id] += elapsed_time;
    total_bytes_transmitted[cluster_id] += PACKET_SIZE; // 更新传输的总字节数

    // 设置信道为空闲状态
    clusters[cluster_id].channel.state = 0;
}

// 每一轮或模拟结束时打印统计数据
void print_statistics(int round_counter) {
    if (round_counter > 0) {
        for (int c = 0; c < NUM_CLUSTERS; ++c) {
            double avg_delay = counts[c] > 0 ? delaytimes[c] / counts[c] : 0;
            double total_delay = counts[c] * avg_delay;
            printf("Cluster %d: Total transmissions: %d, Average delay time: %.6f seconds, Total delay time: %.6f seconds\n",
                   c + 1, counts[c], avg_delay, total_delay);
        }
        printf("-------------------------\n");
    }
}

// 模拟结束后输出最终统计数据
void print_final_statistics() {
    printf("Final statistics after the entire simulation:\n");
    for (int c = 0; c < NUM_CLUSTERS; ++c) {
        double throughput = total_delay_time[c] > 0 ? (total_bytes_transmitted[c] / total_delay_time[c]) : 0;
        printf("Cluster %d: Total packets sent: %d, Total delay time: %.6f seconds, Total bytes transmitted: %ld, Throughput: %.6f B/s\n",
               c + 1, total_transmissions[c], total_delay_time[c], total_bytes_transmitted[c], throughput);
    }
    printf("-------------------------\n");
}

// 处理整个TDMA通信模拟过程
void simulate_tdma_communication(Cluster clusters[]) {
    int slot_counter = 0; // 跟踪时隙的计数器
    int round_counter = 0; // 跟踪轮次的计数器

    while (slot_counter < TOTAL_TIME_SLOTS) { // 模拟循环
        // 每一轮开始时生成发送欲望并排序
        generate_send_will(clusters);

        // 每个簇内的无人机根据发送欲望依次发送数据
        for (int t = 0; t < NUM_DRONES_PER_CLUSTER; ++t) { // 遍历每个时间槽
            for (int c = 0; c < NUM_CLUSTERS; ++c) { // 遍历所有簇
                if (clusters[c].channel.state == 0 && t < NUM_DRONES_PER_CLUSTER) {
                    send_data(&clusters[c].drones[t], c, clusters);
                } else if (t < NUM_DRONES_PER_CLUSTER) {
                    printf("Channel of Cluster %d is currently busy, skipping this transmission.\n", c + 1);
                }
            }

            // 每个时间槽之间等待一段时间
            usleep(TDMA_SLOT_TIME_US);
            slot_counter++;
            
            // 如果达到了总时隙数，结束模拟
            if (slot_counter >= TOTAL_TIME_SLOTS) break;
        }

        round_counter++;
        print_statistics(round_counter);

        // 在每轮结束时重置每个簇的统计信息
        for (int c = 0; c < NUM_CLUSTERS; ++c) {
            counts[c] = 0;
            delaytimes[c] = 0.0;
        }
    }

    // 模拟结束后输出最终统计数据
    print_final_statistics();
    printf("Simulation ended after %d time slots.\n", slot_counter);
}

int main() {
    srand(time(NULL)); // 初始化随机数种子

    Cluster clusters[NUM_CLUSTERS];
    initialize_clusters(clusters);

    // 开始模拟
    simulate_tdma_communication(clusters);

    return 0;
}