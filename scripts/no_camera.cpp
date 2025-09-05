#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
//bibliotecas necessárias. Talvez o opencv.hpp esteja redundante, pq teoricamente ele faz oq o imgproc e o highgui fazem. 

//-----------------------
//        TO-DO
//----------------------

// implementar subtração de gaussian blur pra melhorar a nitidez (?)
// estudar como deixar o código mais leve
// implementar buffer
// limpar o código. Remover formato de kernel e esse tipo de coisa q realmente n tem muito uso

class no_camera {
public:
    no_camera();
    //__init__ do c++
    //esse public é pra qulquer função no codigo acessar, eu tentei usar private no inicio mas tava tendo uns bugs com mutex, que no final eu acabei abandonando tb

    ros::NodeHandle nh_;
    //nodehandle é a api do ros
    ros::Subscriber subscriber_camera_;
    ros::Publisher publisher_controle_;
    cv::Mat imagem_recebida_;
    //estabelece o subscriber, o publisher e cria a matriz pra imagem recebida

    void hsv(const cv::Mat& input, cv::Mat& output);
    void erodir(const cv::Mat& input, cv::Mat& output);
    void dilatar(const cv::Mat& input, cv::Mat& output);
    void processa_imagens(const cv::Mat& input, cv::Mat& output);
    void imagem_callback(const sensor_msgs::ImageConstPtr& msg);
    //estabelece os métodos utilizados no código

    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    int tamanho_kernel_ero_, tamanho_kernel_dil_;
    int forma_kernel_ero_, forma_kernel_dil_;
    //cria variável pras trackbars
};

//----------------------------
//        construtor
//----------------------------

no_camera::no_camera() {
    nh_ = ros::NodeHandle();

    subscriber_camera_ = nh_.subscribe("/camera/rgb/image_raw", 1, &no_camera::imagem_callback, this);
    // "this" faz com que quando o callback seja chamado, ele vai chamar nesse objeto específico, que no caso acaba por ser só esse aqui.
    publisher_controle_ = nh_.advertise<std_msgs::Float32MultiArray>("/info_objeto", 5);
    //puxa pela api do ros a img do gazebo e estabele o publisher com fila de 5

    h_min_ = 0; h_max_ = 179;
    s_min_ = 0; s_max_ = 255;
    v_min_ = 0; v_max_ = 255;

    tamanho_kernel_ero_ = 1, tamanho_kernel_dil_ = 1;
    forma_kernel_ero_ = 0, forma_kernel_dil_ = 0;
    //valor incial pra trackbar do opencv pq ele nao roda as janelas sem

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
    //esse "&" que aparece aqui e outras vezes é pra indicar o endereço da variavel, ao inves de criar uma copia dela pro metodo ler
}

//----------------------------
//         metodos
//----------------------------

void no_camera::hsv(const cv::Mat& input, cv::Mat& output) {
    //esse const é pra indicar pra ele só ler o que o endereço fornecer
    cv::Mat imagem_hsv;
    //cria variavel hsv
    cv::cvtColor(input, imagem_hsv, cv::COLOR_BGR2HSV);
    //joga imagem pra hsv
    cv::inRange(imagem_hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), output);
    //mascara a danada
}

void no_camera::erodir(const cv::Mat& input, cv::Mat& output) {
    int tipo = (forma_kernel_ero_ == 0 ? cv::MORPH_RECT : forma_kernel_ero_ == 1 ? cv::MORPH_CROSS : cv::MORPH_ELLIPSE);
    //esse "?" e ":" é um if else compacto. Lê: if forma == 0 -> rect, else if forma = 1 ...
    cv::Mat k = cv::getStructuringElement(tipo, cv::Size(2*tamanho_kernel_ero_+1, 2*tamanho_kernel_ero_+1),
                                          cv::Point(tamanho_kernel_ero_, tamanho_kernel_ero_));
    //estrutura de kernel que nem no python
    cv::erode(input, output, k);
}

void no_camera::dilatar(const cv::Mat& input, cv::Mat& output) {
    int tipo = (forma_kernel_dil_ == 0 ? cv::MORPH_RECT : forma_kernel_dil_ == 1 ? cv::MORPH_CROSS : cv::MORPH_ELLIPSE);
    cv::Mat k = cv::getStructuringElement(tipo, cv::Size(2*tamanho_kernel_dil_+1, 2*tamanho_kernel_dil_+1),
                                          cv::Point(tamanho_kernel_dil_, tamanho_kernel_dil_));
    cv::dilate(input, output, k);
}

void no_camera::processa_imagens(const cv::Mat& input, cv::Mat& output) {
    //esse daqui é so os metodos ordenados pra n ter que adicionar 4 linhas de atribuiçao no main
    cv::Mat filtrada, erodida, dilatada;
    hsv(input, filtrada);
    erodir(filtrada, erodida);
    dilatar(erodida, dilatada);
    output = dilatada;
}

void no_camera::imagem_callback(const sensor_msgs::ImageConstPtr& msg) {
    imagem_recebida_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    // a setinha é pq ele não retorna uma matriz do opencv, mas um objeto do tipo cvimage. Esse objeto tem uma entrada chamada img que é a matriz que queremos
}

//----------------------------
//          main
//----------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "no_camera");
    //eu nao entendi direito esse agrc, argv e char**. sei que um é o numero de argumentos, o outro o vetor e aparentemente
    //char** é pra indicar que é um array de strings, mas eu não sei exatamente que argumentos nem que array é esse, mas o ROS precisa pra rodar
    no_camera node;
    //esse inicia o nó de fato

    ros::Rate loop_rate(30);
    //esse cara não deveria existir conforme orientaçoes. mas eu quero ele aqui pro meu computador não pegar fogo. 
    cv::Mat frame, imagem_original, imagem_filtrada;

    while (ros::ok()) {
        ros::spinOnce();
        //pra ele rodar uma vez por cada ciclo

        std_msgs::Float32MultiArray msg_publicacao;
        msg_publicacao.data = {0.0, 0.0, 0.0, 0.0};
        //cria a msg com os 4 floats vazios pra publicar msm quando n ve nada

        {
            if (!node.imagem_recebida_.empty()) frame = node.imagem_recebida_.clone();
            //buffer. Ele copia a ultima imagem recebida se nao for vazia. Aqui em baixo, ele sempre vai rodar pq frame nunca vai ser vazio, mas pode ser repetido
        }

        if (!frame.empty()) {
            //"!" é diferente que nem no python, mas sem o =
            imagem_original = frame;
            //clona a imagem original pra mostrar
            node.processa_imagens(imagem_original, imagem_filtrada);

            cv::imshow("Imagem Original", imagem_original);
            cv::imshow("Imagem Filtrada", imagem_filtrada);
            cv::waitKey(1);

            std::vector<std::vector<cv::Point>> contornos;
            cv::findContours(imagem_filtrada.clone(), contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            //cria o array com os contornos e preenche ele

            float altura_imagem = imagem_original.rows;
            float largura_imagem = imagem_original.cols;
            //aqui n da pra puxar isso direto com .shape, ent vc puxa linhas e colunas da matriz, e cada uma é um pixel

            if (!contornos.empty()) {
                size_t maior = 0;
                //esse é o indice do maior contorno. eu começo dizendo que ele ta no 0
                double area_max = cv::contourArea(contornos[0]);
                for (size_t i=1; i<contornos.size(); i++) {
                    double a = cv::contourArea(contornos[i]);
                    if (a > area_max) { area_max = a; maior = i; }
                }
                //esse é basicamente o max do python. Ele vai comparando ate o final do array pra ver qual o maior contorno. Talvez tenha como otimizar isso

                cv::Moments m = cv::moments(contornos[maior]);
                //moments goated né, preciso nem falar
                float cx = m.m10/m.m00;
                float area_obj = m.m00;

                msg_publicacao.data[0] = cx;
                msg_publicacao.data[1] = area_obj;
                //joga o cx e a area pra mensagem.
            }

            msg_publicacao.data[2] = imagem_original.rows;
            msg_publicacao.data[3] = imagem_original.cols;
            //joga a altura e a largura pra msg

            node.imagem_recebida_.release();
            //isso basicamente limpa os dados da matriz imagem recebida pra jogar as que a função receber no proximo ciclo. Ajuda a poupar memória, mas deve ser algo bem mixuruca
        }

        node.publisher_controle_.publish(msg_publicacao);
        //sempre publica a msg
        loop_rate.sleep();
        //relaxa depois de cumprir o ciclo do loop
    }

    return 0;
}