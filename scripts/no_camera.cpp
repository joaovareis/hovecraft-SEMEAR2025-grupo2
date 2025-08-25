#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

class no_camera {
public:
    no_camera();  // Construtor sem ros::init
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_camera_;
    ros::Publisher publisher_controle_;
    cv::Mat imagem_recebida_;

    void hsv(const cv::Mat& input, cv::Mat& output);
    void erodir(const cv::Mat& input, cv::Mat& output);
    void dilatar(const cv::Mat& input, cv::Mat& output);
    void processa_imagens(const cv::Mat& input, cv::Mat& output);
    void imagem_callback(const sensor_msgs::ImageConstPtr& msg);

    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    int tamanho_kernel_ero_, tamanho_kernel_dil_;
    int forma_kernel_ero_, forma_kernel_dil_;
};

//----------------------------
// Construtor
//----------------------------
no_camera::no_camera() {
    nh_ = ros::NodeHandle();

    subscriber_camera_ = nh_.subscribe("/camera/rgb/image_raw", 1, &no_camera::imagem_callback, this);
    publisher_controle_ = nh_.advertise<std_msgs::Float32MultiArray>("/info_objeto", 5);

    h_min_ = 0; h_max_ = 179;
    s_min_ = 0; s_max_ = 255;
    v_min_ = 0; v_max_ = 255;

    tamanho_kernel_ero_ = 1, tamanho_kernel_dil_ = 1;
    forma_kernel_ero_ = 0, forma_kernel_dil_ = 0;

    cv::namedWindow("Ajuste Filtro");
    cv::createTrackbar("h_min", "Ajuste Filtro", &h_min_, 179);
    cv::createTrackbar("h_max", "Ajuste Filtro", &h_max_, 179);
    cv::createTrackbar("s_min", "Ajuste Filtro", &s_min_, 255);
    cv::createTrackbar("s_max", "Ajuste Filtro", &s_max_, 255);
    cv::createTrackbar("v_min", "Ajuste Filtro", &v_min_, 255);
    cv::createTrackbar("v_max", "Ajuste Filtro", &v_max_, 255);
    cv::createTrackbar("ERO_forma", "Ajuste Filtro", &forma_kernel_ero_, 2);
    cv::createTrackbar("ERO_kernel", "Ajuste Filtro", &tamanho_kernel_ero_, 2);
    cv::createTrackbar("DIL_forma", "Ajuste Filtro", &forma_kernel_dil_, 2);
    cv::createTrackbar("DIL_kernel", "Ajuste Filtro", &tamanho_kernel_dil_, 2);
}

//----------------------------
// Métodos de processamento
//----------------------------
void no_camera::hsv(const cv::Mat& input, cv::Mat& output) {
    cv::Mat imagem_hsv;
    cv::cvtColor(input, imagem_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(imagem_hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), output);
}

void no_camera::erodir(const cv::Mat& input, cv::Mat& output) {
    int tipo = (forma_kernel_ero_ == 0 ? cv::MORPH_RECT : forma_kernel_ero_ == 1 ? cv::MORPH_CROSS : cv::MORPH_ELLIPSE);
    cv::Mat k = cv::getStructuringElement(tipo, cv::Size(2*tamanho_kernel_ero_+1, 2*tamanho_kernel_ero_+1),
                                          cv::Point(tamanho_kernel_ero_, tamanho_kernel_ero_));
    cv::erode(input, output, k);
}

void no_camera::dilatar(const cv::Mat& input, cv::Mat& output) {
    int tipo = (forma_kernel_dil_ == 0 ? cv::MORPH_RECT : forma_kernel_dil_ == 1 ? cv::MORPH_CROSS : cv::MORPH_ELLIPSE);
    cv::Mat k = cv::getStructuringElement(tipo, cv::Size(2*tamanho_kernel_dil_+1, 2*tamanho_kernel_dil_+1),
                                          cv::Point(tamanho_kernel_dil_, tamanho_kernel_dil_));
    cv::dilate(input, output, k);
}

void no_camera::processa_imagens(const cv::Mat& input, cv::Mat& output) {
    cv::Mat filtrada, erodida, dilatada;
    hsv(input, filtrada);
    erodir(filtrada, erodida);
    dilatar(erodida, dilatada);
    output = dilatada;
}

void no_camera::imagem_callback(const sensor_msgs::ImageConstPtr& msg) {
    imagem_recebida_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
}

//----------------------------
// Main
//----------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "no_camera");
    no_camera node;

    ros::Rate loop_rate(30);
    cv::Mat imagem_original, imagem_filtrada;

    while (ros::ok()) {
        ros::spinOnce();

        // Prepara mensagem com valores padrão 0.0
        std_msgs::Float32MultiArray msg_publicacao;
        msg_publicacao.data = {0.0, 0.0, 0.0, 0.0};

        if (!node.imagem_recebida_.empty()) {
            imagem_original = node.imagem_recebida_.clone();
            node.processa_imagens(imagem_original, imagem_filtrada);

            cv::imshow("Imagem Original", imagem_original);
            cv::imshow("Imagem Filtrada", imagem_filtrada);
            cv::waitKey(1);

            std::vector<std::vector<cv::Point>> contornos;
            cv::findContours(imagem_filtrada.clone(), contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            float altura_imagem = imagem_original.rows;
            float largura_imagem = imagem_original.cols;

            if (!contornos.empty()) {
                // Pega o maior contorno
                size_t maior = 0;
                double area_max = cv::contourArea(contornos[0]);
                for (size_t i=1; i<contornos.size(); i++) {
                    double a = cv::contourArea(contornos[i]);
                    if (a > area_max) { area_max = a; maior = i; }
                }
                cv::Moments m = cv::moments(contornos[maior]);
                float cx = m.m10/m.m00;
                float area_obj = m.m00;

                // Atualiza a mensagem
                msg_publicacao.data[0] = cx;
                msg_publicacao.data[1] = area_obj;
            }

            // Sempre publica altura e largura da imagem
            msg_publicacao.data[2] = imagem_original.rows;
            msg_publicacao.data[3] = imagem_original.cols;

            node.imagem_recebida_.release();
        }

        node.publisher_controle_.publish(msg_publicacao);
        loop_rate.sleep();
    }

    return 0;
}