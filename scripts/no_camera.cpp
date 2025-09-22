#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <deque>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <vector>

//bibliotecas necessárias. Talvez o opencv.hpp esteja redundante, pq teoricamente ele faz oq o imgproc e o highgui fazem. 

class no_camera {
public:
//tudo publico msm fds
    no_camera();
    //declara o construtor

    ros::NodeHandle nh_;
    //nodehandle é a api do ros
    
    ros::Subscriber subscriber_camera_;
    ros::Subscriber subscriber_trackbars_;
    ros::Publisher publisher_controle_;
    //subscriber publisher

    cv::Mat imagem_recebida_, imagem_filtrada;
    bool janelas_abertas;
    std::vector<std::vector<cv::Point>> contornos;
    //declara variaveis opencv, bool de checagem e vetor dos contornos pro minmax
    
    void hsv(const cv::Mat& input, cv::Mat& output);
    void erodir(const cv::Mat& input, cv::Mat& output);
    void dilatar(const cv::Mat& input, cv::Mat& output);
    void processa_imagens(const cv::Mat& input, cv::Mat& output);
    void trackbars_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void imagem_callback(const sensor_msgs::ImageConstPtr& msg);
    //estabelece os métodos utilizados no código

    int h_min_ = 0; int h_max_ = 179;
    int s_min_ = 0; int s_max_ = 255;
    int v_min_ = 0; int v_max_ = 255;

    int tamanho_kernel_ero_ = 1; int tamanho_kernel_dil_ = 1;
    int forma_kernel_ero_ = 0;   int forma_kernel_dil_ = 0;
    //variaveis dos filtros

    const size_t tamanho_max_buffer = 10;
    std::deque<cv::Mat> buffer;
    //buffer do pedro

    std_msgs::Float32MultiArray msg_publicacao;
    //msg a ser publicada
};

//----------------------------
//        construtor
//----------------------------

no_camera::no_camera() {

    nh_ = ros::NodeHandle();

    subscriber_camera_ = nh_.subscribe("/camera/rgb/image_raw", 1, &no_camera::imagem_callback, this);
    subscriber_trackbars_ = nh_.subscribe("/info_sliders", 1, &no_camera::trackbars_callback, this);
    // "this" faz com que quando o callback seja chamado, ele vai chamar nesse objeto específico, que no caso acaba por ser só esse aqui.
    publisher_controle_ = nh_.advertise<std_msgs::Float32MultiArray>("/info_objeto", 5);
    //puxa pela api do ros a img do gazebo e estabele o publisher com fila de 5
    //esse "&" que aparece aqui e outras vezes é pra indicar o endereço da variavel, ao inves de criar uma copia dela pro metodo ler

    cv::namedWindow("Imagem Original", cv::WINDOW_NORMAL);
    cv::namedWindow("Imagem Filtrada", cv::WINDOW_NORMAL);
    //por algum motivo o opencv não mostra as imagens no meu pc se n criar a janela no objeto antes

    janelas_abertas = true;
    //bool pra dizer se deve abrir a janela ou n

}

//----------------------------
//         metodos
//----------------------------

void no_camera::hsv(const cv::Mat& input, cv::Mat& output) {
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
    hsv(input, output);
    erodir(output, output);
    dilatar(output, output);
}

void no_camera::imagem_callback(const sensor_msgs::ImageConstPtr& msg) {
    buffer.push_back(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
    // a setinha é pq ele não retorna uma matriz do opencv, mas um objeto do tipo cvimage. Esse objeto tem uma entrada chamada img que é a matriz que queremos
    if (buffer.size() > tamanho_max_buffer) {

        buffer.pop_front();
        //checa se ja estamos no tamanho max do buffer, caso afirmativo se livra do elemento mais antiga
    }
}

void no_camera::trackbars_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    
    h_min_ = msg->data[0];
    h_max_ = msg->data[1];
    s_min_ = msg->data[2];
    s_max_ = msg->data[3];
    v_min_ = msg->data[4];
    v_max_ = msg->data[5];
    tamanho_kernel_ero_ = msg->data[6];
    forma_kernel_ero_ = msg->data[7];
    tamanho_kernel_dil_ = msg->data[8];
    forma_kernel_dil_ = msg->data[9];
    //eu tlg se eu copiasse um vetor e só referenciasse ele nos métodos talvez seria marginalmente mais rapido, mas mudar tudo no codigo pra isso é meio chato

}

//----------------------------
//          main
//----------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "no_camera");

    no_camera objeto_classe;
    ros::Rate loop_rate(30);

    while (ros::ok()) {

        ros::spinOnce();
        //pra ele rodar uma vez por cada ciclo
        objeto_classe.msg_publicacao.data = {0.0, 0.0, 0.0, 0.0};
        //cria a msg com os 4 floats vazios pra publicar msm quando n ve nada

        if (!objeto_classe.buffer.empty()) {
            //"!" é diferente que nem no python, mas sem o =
            const cv::Mat& imagem_original_ref = objeto_classe.buffer.back();
            //referencia a imagem original

            objeto_classe.processa_imagens(imagem_original_ref, objeto_classe.imagem_filtrada);

            if (objeto_classe.janelas_abertas) {
                //mostra as imagens na janela se o bool for true
                cv::imshow("Imagem Original", imagem_original_ref);
                cv::imshow("Imagem Filtrada", objeto_classe.imagem_filtrada);
            }

            int key = cv::waitKey(1);
            //registra a tecla apertada

            if (key == 'q') {
                //se for q de quit, torna o bool false e fecha td. Teoricamente da pra apertar q denovo e abrir, mas precisa selecionar uma janela do opencv pra isso
                //dai só criando uma janela "mestre" que sempre ta aberta pra vc selecionar
                objeto_classe.janelas_abertas = !objeto_classe.janelas_abertas;
                cv::destroyAllWindows();
            }

            objeto_classe.contornos.clear();
            cv::findContours(objeto_classe.imagem_filtrada, objeto_classe.contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            //zera o array com os contornos e preenche ele

            if (!objeto_classe.contornos.empty()) {
                size_t maior = 0;
                //esse é o indice do maior contorno. eu começo dizendo que ele ta no 0
                double area_max = cv::contourArea(objeto_classe.contornos[0]);
                for (size_t i=1; i<objeto_classe.contornos.size(); i++) {
                    double a = cv::contourArea(objeto_classe.contornos[i]);
                    if (a > area_max) { area_max = a; maior = i; }
                }
                //esse é basicamente o max do python. Ele vai comparando ate o final do array pra ver qual o maior contorno.
                //agora que tem a biblioteca do algorithm podia implementar ela, mas aparentemente as duas sao lineares ent n muda eficiencia

                const std::vector<cv::Point>& maior_contorno = objeto_classe.contornos[maior];
                //tem que refenciar como vector se nao o algorithm não le cv point

                auto iterador_x = std::minmax_element(maior_contorno.begin(), maior_contorno.end(),
                                        [](const cv::Point& a, const cv::Point& b) {
                                            return a.x < b.x;   
                                        });
                //auto pq o nome da variavel é quase uma linha inteira mano
                //ele roda o vetor do maior contorno pra encontrar o point com maior e menor x. retorna o endereço do point

                cv::Point extrema_esquerda = *(iterador_x.first);
                cv::Point extrema_direita = *(iterador_x.second);
                //aqui pega o endereço do point transforma de referencia em variável

                float cx = (extrema_esquerda.x + extrema_direita.x)/2;
                //calcula o centro do maior contorno pela média entre os valores de x

                objeto_classe.msg_publicacao.data[0] = cx;
                objeto_classe.msg_publicacao.data[1] = area_max;
                //joga o cx e a area pra mensagem.
            }

            objeto_classe.msg_publicacao.data[2] = imagem_original_ref.rows;
            objeto_classe.msg_publicacao.data[3] = imagem_original_ref.cols;
            //joga a altura e a largura pra msg

            objeto_classe.imagem_recebida_.release();
            //isso basicamente limpa os dados da matriz imagem recebida pra jogar as que a função receber no proximo ciclo. Ajuda a poupar memória, mas deve ser algo bem mixuruca
        }

        objeto_classe.publisher_controle_.publish(objeto_classe.msg_publicacao);
        //sempre publica a msg
        loop_rate.sleep();
        //relaxa depois de cumprir o ciclo do loop
    }

    cv::destroyAllWindows();
    return 0;
}