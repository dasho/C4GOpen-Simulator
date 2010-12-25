/**
***
*** Universidade Federal de Minas Gerais             - UFMG
*** Grupo de Desenvolvimento de Veículos Autônomos   - PDVA
*** Laboratório de Sistemas de Computação e Robótica - CORO
***
*** Adriano de Araújo Abreu Mourão ( adrianodasho@ufmg.br )
*** Graduando em Engenharia de Controle e Automação
***
*** C4G_conecta.cpp
***
*** Função MEX (mais informações digite "help mex" no MatLab) que cria a conexão rtnet para o simulador
*** do robô SmartSix.
***
**/

#include <iostream>
#include <iomanip>
#include <fstream>

#include <pthread.h>

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sched.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>

// biblitecas para criar conexão rtnet;
#include <rtai_lxrt.h>
#include <rtnet.h>
#include <rtdm/rtdm.h>

// biblioteca C4GOpen que contém o formato dos pacotes de comunicação
#include <C4gOpen.hpp>

#define C4G_CLIENT_PORT     3600
#define C4G_SERVER_PORT     1000
#define C4G_SERVER_ADDR     "127.0.0.1"
#define GBM_NAX_ARM         6
#define MAX_INFO            20
#define KEY                123
#define ALLOWED            1
#define NOT_ALLOWED        0
#define SEM0               0
#define SEM1               1
#define SEM2               2
#define CLIENT_TIMEOUT	   10000000000LL
#define NUM_THREADS        1

using namespace std;

static struct sockaddr_in local_addr;
static struct sockaddr_in server_addr;

union semun {
              int val ;
              struct semid_ds buf[2] ;
              unsigned short int array[4] ;
              struct seminfo *__buf;
};

/* thread instance; */
void *SetValues(void *threadid)
{
    long tid;
    tid = (long)threadid;
    cout << "Hello, World\n";
}

int main()
{
    struct InitPacket initPack;
    struct CommPacketRx pacote;
    struct CommPacketTx dados_recebidos;

    ifstream fin;
    ofstream fout;

    pthread_t threads[NUM_THREADS];
    int rc,t;

    long long c_timeout(CLIENT_TIMEOUT);
    int i, sockfd, ret, inewCS = 10;

    bool keepgoing = 1;
    char info[MAX_INFO];
    int numberOfOpenAx;

    int semid, iosemval, onsemval, setsemval;
    char path[] = "semaphoros1";
    union semun ioarg, onarg, setarg;

    RT_TASK *lxrtnettsk;

    /* set variables values; */
    sockfd = 0;
    ret    = 0;

    /* set address structures to zero; */
    memset(&local_addr, 0, sizeof(struct sockaddr_in));
    memset(&server_addr, 0, sizeof(struct sockaddr_in));

    // set addresses;
    local_addr.sin_family       = AF_INET;
    local_addr.sin_addr.s_addr  = INADDR_ANY;
    local_addr.sin_port         = htons(C4G_CLIENT_PORT);

    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(C4G_SERVER_PORT);
    inet_aton(C4G_SERVER_ADDR,&(server_addr.sin_addr));

    /* Lock allocated memory into RAM. */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // create new socket
    if((sockfd = rt_dev_socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror( "Erro ao iniciar socket");
        exit(1);
    }

    // initialize a real time buddy;
    lxrtnettsk = rt_task_init(4800, 1, 0, 0);
    if(lxrtnettsk == NULL)
    {
        rt_dev_close(sockfd);
        perror( "Nao conseguiu iniciar uma tarefa real time");
        exit(1);
    }

    /* switch over hard real time mode; */
    rt_make_hard_real_time();

    /* bind socket to local address; */
    ret = rt_dev_bind(sockfd, (struct sockaddr *)&local_addr, sizeof(struct sockaddr_in));

    //ioctl_rt(sockfd, RTNET_RTIOC_TIMEOUT, &nanoseconds);
    rt_dev_ioctl(sockfd, RTNET_RTIOC_TIMEOUT, &c_timeout);

    cout << "** \n";
    cout << "** Simulador SmartSix - C4GOpen;\n";
    cout << "** Interface RTnet.\n";
    cout << "** \n";
    cout << "** Universidade Federal de Minas Gerais                        - UFMG\n";
    cout << "** Grupo de Pesquisa e Desenvolvimento de Veículos Autônomos   - PDVA\n";
    cout << "** Laboratório de Sistemas de Computação e Robótica            - CORO\n";
    cout << "** \n";
    cout << "** autor:\n";
    cout << "** Adriano de Araújo Abreu Mourão\n";
    cout << "** (adrianodasho@ufmg.br)\n";
    cout << "** \n";
    cout << "** \n";
    cout << "** Esse programa deve ser iniciado após o servidor - c4gopen e antes \n";
    cout << "** do simulador MatLab + RobotToolbox\n";
    cout << "** \n";

    cout << "-- conectando ao servidor... \n";

    /* Specify destination address for socket; needed for rt_socket_send(). */
    rt_dev_connect(sockfd, (struct sockaddr *) &server_addr,sizeof(struct sockaddr_in));
    cout << "-- conectado.\n";

    /* create 3 semaphores; */
    cout << "-- criando três semaphoros;\n ";
    if (( semid = semget(ftok(path,(key_t)KEY), 3, IPC_CREAT|IPC_EXCL|0600)) == -1)
    {
        cout << "-- semaphoros já existentes; recuperando ID;\n";
        if (( semid = semget(ftok(path,(key_t)KEY),0,0))==-1)
        {
          perror("impossivel achar o conjunto de semaforos") ;
          exit(1) ;
        }
        cout << "-- IDs recuperados com sucesso. ID = " << semid << "\n";
    }
    else
        cout << "-- sucesso;\n";

    /* set semaphoros values to ALLOWED, NOT ALLOWED and ALLOWED; */
    ioarg.val = ALLOWED ;
    if ((semctl(semid,SEM0,SETVAL,ioarg)) == -1)
    {
         perror("Error semctl") ;
         exit(1);
    }

    onarg.val = NOT_ALLOWED ;
    if ((semctl(semid,SEM1,SETVAL,onarg)) == -1)
    {
         perror("Error semctl") ;
         exit(1);
    }

    setarg.val = ALLOWED ;
    if ((semctl(semid,SEM2,SETVAL,setarg)) == -1)
    {
         perror("Error semctl") ;
         exit(1);
    }

    cout << "-- comunicando; \n";

    /* create threads; */
    for(t = 0; t < NUM_THREADS; t++)
    {
        rc = pthread_create(&threads[t], NULL, SetValues, (void *)t);
        if(rc)
        {
            cout << "----- ERRO, nao conseguiu criar uma thread;\n";
            exit(1);
        }
    }

    while(keepgoing)
    {
        if ( (iosemval = semctl(semid,SEM0,GETVAL,ioarg)) == -1)
        {
          perror("Error semctl() GETVAL") ;
          exit(1) ;
        }

        if ( (onsemval = semctl(semid,SEM1,GETVAL,onarg)) == -1)
        {
          perror("Error semctl() GETVAL") ;
          exit(1) ;
        }

        /* se o semaphoro pode ser utilizado; */
        if((iosemval == ALLOWED) && (onsemval == ALLOWED))
        {
            /* altera o valor do semaphoro; */
            ioarg.val = NOT_ALLOWED ;
            if ((semctl(semid,SEM0,SETVAL,ioarg))==-1)
            {
                 perror("Error semctl") ;
                 exit(1);
            }

            /* abre o arquivo; */
            fin.open ("entrada.txt");

            if(fin.is_open() )
            {
                /* Coloca as novas informações no pacote a ser enviado;
                   Aqui é garantido que se inewCS é igual a 10 não é setado no pacote comum */
                if( inewCS != 10)
                {
                    pacote.header.seqNumber = inewCS;
                    pacote.header.status = DRIVE_ON;
                    pacote.header.functionality = C4G_OPEN_MODE_0;

                    for(i = 0; i < numberOfOpenAx; i++)
                    {
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].mode = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].targetPosition = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].targetVelocity = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].actualPosition = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].actualVelocity = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].targetCurrent = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].extra1 = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].extra2 = atof(info);
                        fin.getline(info,MAX_INFO);
                        pacote.axisData[i].extra3 = atof(info);
                    }

                    inewCS++;
                }
                else
                {
                    initPack.seqNumber = 10;
                    initPack.numberOfFieldsPerPacket = 5*9;
                    initPack.numberOfFieldsPerAxis = 9;
                    fin.getline(info,MAX_INFO);
                    initPack.numberOfAxesInOpenMode = atoi(info);

                    numberOfOpenAx = initPack.numberOfAxesInOpenMode;

                    fin.getline(info,MAX_INFO);
                    initPack.typeOfHeaderField1 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfHeaderField2 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfHeaderField3 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField1 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField2 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField3 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField4 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField5 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField6 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField7 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField8 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.typeOfAxisDataField9 = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.sampleTime = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.majorNumber = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.minorNumber = atoi(info);
                    fin.getline(info,MAX_INFO);
                    initPack.buildNumber = atoi(info);

                    for(i = 0; i < MAX_NUM_AXES_PER_ARM; i++)
                    {
                        fin.getline(info,MAX_INFO);
                        initPack.arm1OpenMode[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2OpenMode[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3OpenMode[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4OpenMode[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1OpenAxesMap[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2OpenAxesMap[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3OpenAxesMap[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4OpenAxesMap[i] = atoi(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1CalibrationConstant[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2CalibrationConstant[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3CalibrationConstant[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4CalibrationConstant[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1CurrentLimit[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2CurrentLimit[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3CurrentLimit[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4CurrentLimit[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1KinInflCoeff[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2KinInflCoeff[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3KinInflCoeff[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4KinInflCoeff[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1TxRate[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2TxRate[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3TxRate[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4TxRate[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm1FollowingError[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm2FollowingError[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm3FollowingError[i] = atof(info);
                        fin.getline(info,MAX_INFO);
                        initPack.arm4FollowingError[i] = atof(info);
                    }

                    inewCS++;
                }

                /* Send a message; */
                if(inewCS == 11)
                    rt_dev_sendto(sockfd,&initPack, sizeof(struct InitPacket), 0,(struct sockaddr *) &server_addr,sizeof(struct sockaddr_in));
                else
                    rt_dev_sendto(sockfd,&pacote, sizeof(struct CommPacketRx), 0,(struct sockaddr *) &server_addr,sizeof(struct sockaddr_in));
                /* recebe o pacote UDP; */
                ret = rt_dev_recv(sockfd,&dados_recebidos,sizeof(struct CommPacketTx), 0);
                /* é necessário entender o que é esse pacote com 192 bytes */
                if((ret != (sizeof(struct Header)+ numberOfOpenAx*sizeof(AxisDataRx))) && (ret != sizeof(struct CommPacketTx)))
                    keepgoing = 0;

                fout.open("saida.txt");

                /* retorna valores ao Matlab; */

                for(i = 0; i < numberOfOpenAx; i++)
                {
                    fout << setprecision(10) << dados_recebidos.axisData[i].mode << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].targetPosition << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].targetVelocity << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].measure << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].ffwVelocity << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].ffwCurrent << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].extra1 << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].extra2 << endl;
                    fout << setprecision(10) << dados_recebidos.axisData[i].extra3 << endl;
                }

                fout.close();
                fin.close();
            }
            sleep(1);
             /* altera o valor do semaphoro; */
            ioarg.val = ALLOWED ;
            if ((semctl(semid,SEM0,SETVAL,ioarg))==-1)
            {
                 perror("Error semctl") ;
                 exit(1);
            }

        } /* fim do if semaphoro; */
        else
            sleep(1);
    }

    /* switch over to soft real time mode; */
    rt_make_soft_real_time();

    /* close socket; */
    rt_dev_close(sockfd);

    /* delete real time buddy; */
    rt_task_delete(lxrtnettsk);

    /* destroy thread; */
    pthread_exit(NULL);

    if (semctl(semid,0,IPC_RMID,0) == -1)
    {
          perror("problema durante a destruicao dos semaforos") ;
          exit(1) ;
    }

    return 0;
}
