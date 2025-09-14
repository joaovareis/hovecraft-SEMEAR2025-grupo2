#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>


class no_trackbars {
    public:

    no_trackbars();

    ros::NodeHandle nh_;
    ros::Publisher publisher_trackbars_;

    void cria_e_publica_mensagem();

    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    int tamanho_kernel_ero_, tamanho_kernel_dil_;
    int forma_kernel_ero_, forma_kernel_dil_;

};

//---------------------------
//         construtor
//---------------------------

no_trackbars::no_trackbars() {
    //matando uma formiga com uma .38 criando uma classe pra esse nó

    publisher_trackbars_ = nh_.advertise<std_msgs::Float32MultiArray>("/info_sliders", 5);
    //so um publisher

    h_min_ = 0; h_max_ = 179;
    s_min_ = 0; s_max_ = 255;
    v_min_ = 0; v_max_ = 255;

    tamanho_kernel_ero_ = 1, tamanho_kernel_dil_ = 1;
    forma_kernel_ero_ = 0, forma_kernel_dil_ = 0;
    //todas as variaveis do opencv

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
    //trackbars do opencv

};

//---------------------------
//         metodos
//---------------------------

void no_trackbars::cria_e_publica_mensagem() {

    std_msgs::Float32MultiArray info_sliders;
    //cria a msg

    info_sliders.data.resize(10);

    info_sliders.data[0] = h_min_;
    info_sliders.data[1] = h_max_;
    info_sliders.data[2] = s_min_;
    info_sliders.data[3] = s_max_;
    info_sliders.data[4] = v_min_;
    info_sliders.data[5] = v_max_;
    info_sliders.data[6] = tamanho_kernel_ero_;
    info_sliders.data[7] = forma_kernel_ero_;
    info_sliders.data[8] = tamanho_kernel_dil_;
    info_sliders.data[9] = forma_kernel_dil_;
    //preencher a danada

    publisher_trackbars_.publish(info_sliders);
    //manda bala

};

//---------------------------
//      loop principal
//---------------------------

int main(int argc, char** argv) {

    ros::init(argc, argv, "no_trackbars");
    no_trackbars objeto_classe;
    //inicia o no e cria um objeto

    while (ros::ok()) {
        
        objeto_classe.cria_e_publica_mensagem();
        //roda o unico metodo do codigo

        cv::waitKey(1);
        //o opencv nao roda sem isso aq

        if (cv::getWindowProperty("Ajuste Filtro", cv::WND_PROP_VISIBLE <= 0)) {
            //se a janela for fechada ele retorna 0 ou -1 eu nao sei. Dai encerra o nó pq a gente n precisa mais ajustar nd

            ros::shutdown();
        }

    }

    cv::destroyAllWindows();

    return(0);
}

