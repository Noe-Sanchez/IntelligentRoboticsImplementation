#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
// Ill translate this from ROS1 to ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <queue>

#define MPU6050_ADDR 0x68
#define GRAVITY 9.81
#define SAMPLES_FOR_OFFSET 3000
#define SAMPLES_FOR_VARIANCE 1000
#define ACC_SENSITIVITY_FACTOR 16384.0
#define GYRO_SENSITIVITY_FACTOR 131.0
#define GYRO_COEF 0.98f
#define ACC_COEF 0.02f
#define MOVING_AVERAGE_WINDOW 25

/*		MPU6050
				  +X
		-------------------------
		|						|
		|						|
	+Y	|						|	+Z (UP)
		|		  CHIP			|
		|					LED |
		|	      Pins			|
		-------------------------

		SDA: Pin 3 (I2C Bus no. 8)
		SCL: Pin 5
		VCC: Pin 1 (3.3V)
		GND: Pin 9

*/

std::vector<std::vector<double>> dataForOffset;
std::vector<float> variance(10, 0.0);
std::vector<double> data_for_offset;
std::vector<float> mean;
int16_t rawXacc,rawYacc,rawZacc,rawXomg,rawYomg,rawZomg;
float accX=0,accY=0,accZ=0,omgX=0,omgY=0,omgZ=0,angX=0,angY=0,angZ=0;
float angAccX,angAccY,accx,accy,accz,omgx,omgy,omgz,qx=0,qy=0,qz=0,qw=0;
float gyroXOffset,gyroYOffset,gyroZOffset,dt;

class MPU6050 : public rclcpp::Node {
public:
    MPU6050() : Node("mpu6050") {
        RCLCPP_INFO(this->get_logger(), "MPU node started");
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("mpu6050/data", 10);
        yaw_pub = this->create_publisher<std_msgs::msg::Float64>("mpu6050/yaw", 10);
        filtered_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("mpu6050/filtered_yaw", 10);
        moving_average_yaw_pub = this->create_publisher<std_msgs::msg::Float64>("mpu6050/moving_average_yaw", 10);
        rclcpp::WallRate loop_rate(100);
        start_imu();
        RCLCPP_INFO(this->get_logger(), "MPU6050 node running...");
        start = this->get_clock()->now().seconds() * 1000;
        imu_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MPU6050::imu_callback, this));
    }

    void start_imu() {
        if((file = open(bus, O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the bus.");
            return;
        }
        if(ioctl(file, I2C_SLAVE, MPU6050_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to the sensor.");
            return;
        }

        // ---------- Calculating offsets ----------
        RCLCPP_INFO(this->get_logger(), "Calculating offsets...");
        for(int i=0;i<SAMPLES_FOR_OFFSET;i++){
            write(file, config, 2);
            char reg[1] = {0x43}; // GYRO_XOUT_H register
            write(file, reg, 1);
            char data[6] = {0};
            if(read(file, data, 6) != 6) {
                RCLCPP_ERROR(this->get_logger(), "Error: Input/output error\n");
                return;
            }

            // Convert data to omega values
            rawXomg = (data[0] << 8) | data[1];
            rawYomg = (data[2] << 8) | data[3];
            rawZomg = (data[4] << 8) | data[5];

            omgX += rawXomg / GYRO_SENSITIVITY_FACTOR;
            omgY += rawYomg / GYRO_SENSITIVITY_FACTOR;
            omgZ += rawZomg / GYRO_SENSITIVITY_FACTOR;
        }
        gyroXOffset = omgX / SAMPLES_FOR_OFFSET;
        gyroYOffset = omgY / SAMPLES_FOR_OFFSET;
        gyroZOffset = omgZ / SAMPLES_FOR_OFFSET;
        omgX = 0;
        omgY = 0;
        omgZ = 0;
        // ---------- Offsets Calculated ----------

        double start = this->get_clock()->now().seconds() * 1000;
        double end;

        // ---------- Calculating variances ----------
        RCLCPP_INFO(this->get_logger(), "Calculating variances...");
        for(int i=0;i<SAMPLES_FOR_VARIANCE;i++){
            write(file, config, 2);
            char reg[1] = {0x3B}; // ACCEL_XOUT_H register
            write(file, reg, 1);
            char data[14] = {0};
            if(read(file, data, 14) != 14) {
                RCLCPP_ERROR(this->get_logger(), "Error: Input/output error\n");
                return;
            }

            rawXacc = (data[0] << 8) | data[1];
            rawYacc = (data[2] << 8) | data[3];
            rawZacc = (data[4] << 8) | data[5];

            accx = (rawXacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
            accy = (rawYacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
            accz = (rawZacc / ACC_SENSITIVITY_FACTOR * GRAVITY);
            data_for_offset.push_back(accx);
            data_for_offset.push_back(accy);
            data_for_offset.push_back(accz);
            accX += accx;
            accY += accy;
            accZ += accz;

            // Constant gravitational acceleration is there so...
            angAccX = atan2(accy, accz +abs(accx)) * 360 / 2.0 / M_PI;
            angAccY = atan2(accx, accz +abs(accy)) * 360 / -2.0 / M_PI;

            // Convert data to omega values
            rawXomg = (data[8] << 8) | data[9];
            rawYomg = (data[10] << 8) | data[11];
            rawZomg = (data[12] << 8) | data[13];

            omgx = (rawXomg / GYRO_SENSITIVITY_FACTOR) - gyroXOffset;
            omgy = (rawYomg / GYRO_SENSITIVITY_FACTOR) - gyroYOffset;
            omgz = (rawZomg / GYRO_SENSITIVITY_FACTOR) - gyroZOffset;
            data_for_offset.push_back(omgx * (M_PI / 180));
            data_for_offset.push_back(omgy * (M_PI / 180));
            data_for_offset.push_back(omgz * (M_PI / 180));
            omgX += omgx;
            omgY += omgy;
            omgZ += omgz;

            // Calculating angle in degrees
            end = this->get_clock()->now().seconds() * 1000;
            dt = (end - start) / 1000;
            // angX += omgx * dt;
            // angY += omgy * dt;
            angX = (GYRO_COEF * (angX + omgx * dt)) + (ACC_COEF * angAccX);
            angY = (GYRO_COEF * (angY + omgy * dt)) + (ACC_COEF * angAccY);
            angZ += omgx * dt;

            // Degree -> radians -> quaternians
            q.setRPY(angX * (M_PI / 180), angY * (M_PI / 180), angZ * (M_PI / 180));
            tf2::convert(q, q_msg);
            qx += q_msg.x;
            qy += q_msg.y;
            qz += q_msg.z;
            qw += q_msg.w;
            data_for_offset.push_back(q_msg.x);
            data_for_offset.push_back(q_msg.y);
            data_for_offset.push_back(q_msg.z);
            data_for_offset.push_back(q_msg.w);
            dataForOffset.push_back(data_for_offset);

            start = end;
        }

        // Calculate mean
        mean.push_back(accX / SAMPLES_FOR_VARIANCE);
        mean.push_back(accY / SAMPLES_FOR_VARIANCE);
        mean.push_back(accZ / SAMPLES_FOR_VARIANCE);
        mean.push_back(omgX / SAMPLES_FOR_VARIANCE);
        mean.push_back(omgY / SAMPLES_FOR_VARIANCE);
        mean.push_back(omgZ / SAMPLES_FOR_VARIANCE);
        mean.push_back(qx / SAMPLES_FOR_VARIANCE);
        mean.push_back(qy / SAMPLES_FOR_VARIANCE);
        mean.push_back(qz / SAMPLES_FOR_VARIANCE);
        mean.push_back(qw / SAMPLES_FOR_VARIANCE);

        // Calculate variance
        for(int i=0; i<SAMPLES_FOR_VARIANCE;i++){
            for(int j=0;j<10;j++){
                variance[j] += pow(dataForOffset[i][j] - mean[j], 2);
            }
        }
        for(int j=0;j<10;j++){
            variance[j] /= (SAMPLES_FOR_VARIANCE-1);
        }
        data_for_offset.clear();
        dataForOffset.clear();
        mean.clear();

        imu_data.orientation_covariance[0] = variance[6];
        imu_data.orientation_covariance[1] = 0;
        imu_data.orientation_covariance[2] = 0;
        imu_data.orientation_covariance[3] = variance[7];
        imu_data.orientation_covariance[4] = 0;
        imu_data.orientation_covariance[5] = variance[8];
        imu_data.orientation_covariance[6] = variance[9];
        imu_data.orientation_covariance[7] = 0;
        imu_data.orientation_covariance[8] = 0;

        imu_data.angular_velocity_covariance[0] = variance[3];
        imu_data.angular_velocity_covariance[1] = 0;
        imu_data.angular_velocity_covariance[2] = 0;
        imu_data.angular_velocity_covariance[3] = 0;
        imu_data.angular_velocity_covariance[4] = variance[4];
        imu_data.angular_velocity_covariance[5] = 0;
        imu_data.angular_velocity_covariance[6] = 0;
        imu_data.angular_velocity_covariance[7] = 0;
        imu_data.angular_velocity_covariance[8] = variance[5];

        imu_data.linear_acceleration_covariance[0] = variance[0];
        imu_data.linear_acceleration_covariance[1] = 0;
        imu_data.linear_acceleration_covariance[2] = 0;
        imu_data.linear_acceleration_covariance[3] = 0;
        imu_data.linear_acceleration_covariance[4] = variance[1];
        imu_data.linear_acceleration_covariance[5] = 0;
        imu_data.linear_acceleration_covariance[6] = 0;
        imu_data.linear_acceleration_covariance[7] = 0;
        imu_data.linear_acceleration_covariance[8] = variance[2];
        // ---------- Variances Calculated ----------
    }

    void imu_callback() {
        write(file, config, 2);
        char reg[1] = {0x3B}; // ACCEL_XOUT_H register
        write(file, reg, 1);
        char data[14] = {0};
        if(read(file, data, 14) != 14) {
            RCLCPP_ERROR(this->get_logger(), "Error: Input/output error\n");
            return;
        }

        // Convert data to acceleration values
        rawXacc = (data[0] << 8) | data[1];
        rawYacc = (data[2] << 8) | data[3];
        rawZacc = (data[4] << 8) | data[5];

        accX = rawXacc / ACC_SENSITIVITY_FACTOR * GRAVITY;
        accY = rawYacc / ACC_SENSITIVITY_FACTOR * GRAVITY;
        accZ = rawZacc / ACC_SENSITIVITY_FACTOR * GRAVITY;

        // Constant gravitational acceleration is there so...
        angAccX = atan2(accY, accZ +abs(accX)) * 360 / 2.0 / M_PI;
        angAccY = atan2(accX, accZ +abs(accY)) * 360 / -2.0 / M_PI;

        // Convert data to omega values
        rawXomg = (data[8] << 8) | data[9];
        rawYomg = (data[10] << 8) | data[11];
        rawZomg = (data[12] << 8) | data[13];

        omgX = (rawXomg / GYRO_SENSITIVITY_FACTOR) - gyroXOffset;
        omgY = (rawYomg / GYRO_SENSITIVITY_FACTOR) - gyroYOffset;
        omgZ = (rawZomg / GYRO_SENSITIVITY_FACTOR) - gyroZOffset;

        // Calculating angle in degrees
        end = this->get_clock()->now().seconds() * 1000;
        dt = (end - start) / 1000;
        // angX += omgX * dt;
        // angY += omgY * dt;
        angX = (GYRO_COEF * (angX + omgX * dt)) + (ACC_COEF * angAccX);
        angY = (GYRO_COEF * (angY + omgY * dt)) + (ACC_COEF * angAccY);
        angZ += omgZ * dt;

        filtered_yaw_msg.data = filter(angZ);

        // Moving average
        angXQueue.push(angX);
        angYQueue.push(angY);
        angZQueue.push(angZ);
        if (angXQueue.size() > MOVING_AVERAGE_WINDOW) {
            sumX -= angXQueue.front();
            sumY -= angYQueue.front();
            sumZ -= angZQueue.front();
            angXQueue.pop();
            angYQueue.pop();
            angZQueue.pop();
        }
        sumX += angX;
        sumY += angY;
        sumZ += angZ;

        moving_average_yaw_msg.data = sumZ / angZQueue.size();
        
        // Degree -> radians -> quaternians);
        q.setRPY(angX * (M_PI / 180), angY * (M_PI / 180), angZ * (M_PI / 180));
        tf2::convert(q, q_msg);

        // Create Imu message and publish
        imu_data.header.stamp = this->get_clock()->now();   
        imu_data.header.frame_id = "mpu6050_frame";
        imu_data.orientation = q_msg;
        imu_data.linear_acceleration.x = accX;
        imu_data.linear_acceleration.y = accY;
        imu_data.linear_acceleration.z = accZ;
        imu_data.angular_velocity.x = omgX * (M_PI / 180);
        imu_data.angular_velocity.y = omgY * (M_PI / 180);
        imu_data.angular_velocity.z = omgZ * (M_PI / 180);
        imu_pub->publish(imu_data);

        yaw_msg.data = angZ;
        yaw_pub->publish(yaw_msg);
        filtered_yaw_pub->publish(filtered_yaw_msg);
        moving_average_yaw_pub->publish(moving_average_yaw_msg);

        count++;
        start = end;
    }

    float filter(float input) {
        // Almacenar la muestra actual en el arreglo
        samples[sampleIndex] = input;

        // Incrementar el índice de muestra
        sampleIndex = (sampleIndex + 1) % numSamples;

        // Aplicar el filtro
        float filteredValue = b0 * input + b1 * samples[(sampleIndex - 1 + numSamples) % numSamples] + b2 * samples[(sampleIndex - 2 + numSamples) % numSamples]
                            - a1 * samples[(sampleIndex - 1 + numSamples) % numSamples] - a2 * samples[(sampleIndex - 2 + numSamples) % numSamples];

        return filteredValue;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_yaw_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr moving_average_yaw_pub;
    rclcpp::TimerBase::SharedPtr imu_timer;
    int file;
    const char *bus = "/dev/i2c-8";
    char config[2] = {0x6B, 0x00}; // PWR_MGMT_1 register
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;
    sensor_msgs::msg::Imu imu_data;
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 filtered_yaw_msg;
    std_msgs::msg::Float64 moving_average_yaw_msg;
    double start, end;
    float gyroXOffset,gyroYOffset,gyroZOffset,dt;
    float angAccX,angAccY,accx,accy,accz,omgx,omgy,omgz,qx=0,qy=0,qz=0,qw=0;
    float accX=0,accY=0,accZ=0,omgX=0,omgY=0,omgZ=0,angX=0,angY=0,angZ=0;
    float variance[10] = {0.0};
    std::vector<std::vector<double>> dataForOffset;
    std::vector<double> data_for_offset;
    std::vector<float> mean;
    int16_t rawXacc,rawYacc,rawZacc,rawXomg,rawYomg,rawZomg;
    int count = 0;        

    std::queue<float> angXQueue, angYQueue, angZQueue;
    float sumX = 0, sumY = 0, sumZ = 0;

    const static int numSamples = 10; // Número de muestras para el filtro
    int samples[numSamples]; // Arreglo para almacenar las muestras
    int sampleIndex = 0; // Índice actual en el arreglo de muestras

    // Coeficientes del filtro Butterworth (2º orden, fc = 0.1 Hz)
    const float b0 = 0.02465375f;
    const float b1 = 0.0493075f;
    const float b2 = 0.02465375f;
    const float a1 = -1.9873645f;
    const float a2 = 0.98747212f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050>());
    rclcpp::shutdown();
    return 0;
}