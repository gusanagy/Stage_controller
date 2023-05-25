# Stage_controller Script de Controle de Navegação ROS
Este é um script de controle de navegação para um robô no ROS. Ele utiliza o Python e as bibliotecas ROS para controlar o movimento do robô em direção a um objetivo definido.

## Requisitos
* ROS instalado no sistema
* Bibliotecas ROS: rospy, geometry_msgs, sensor_msgs, nav_msgs
* Python 3

## Execução
* Certifique-se de ter o ROS instalado corretamente no sistema.
* Copie o script para o seu ambiente de trabalho.
* Abra um terminal e execute o seguinte comando para iniciar o ROS:
Copy code
 *roslaunch stage_controller launcher.launch 

## Funcionalidades
O script possui as seguintes funcionalidades:

* Define um objetivo de destino para o robô.
* Inicializa os nós necessários no ROS.
* Recebe informações de odometria e leitura do sensor laser através de subscribers.
* Publica comandos de velocidade para controlar o movimento do robô através de um publisher.
* Calcula a posição atual do robô e a diferença em relação ao objetivo.
* Controla a velocidade linear e angular do robô para movê-lo em direção ao objetivo.
* Detecta obstáculos próximos usando as leituras do sensor laser.
*- Encerra o script quando o objetivo é alcançado.

### Personalização do Objetivo
Você pode personalizar o objetivo alterando as coordenadas x e y na função GoTo_goal(goal_x, goal_y). Por exemplo, para definir um objetivo em x=3.0 e y=7.0, você pode chamar a função da seguinte maneira:

python code
stage_robot.GoTo_goal(goal_x=3.0, goal_y=7.0)
Certifique-se de ajustar as coordenadas de acordo com o ambiente de execução.

### Observações
O script utiliza o tópico /base_pose_ground_truth para obter a posição absoluta do robô e o tópico /base_scan para obter dados do sensor laser. Certifique-se de que esses tópicos estejam configurados corretamente no ambiente de execução.
O script foi projetado para mover o robô em um ambiente de simulação, onde um objetivo é definido e o robô navega em direção a ele. Se estiver executando em um ambiente real, certifique-se de configurar corretamente os tópicos e a comunicação com o robô físico.
Nota: Este README fornece uma visão geral simplificada do script de controle de navegação ROS. Para obter informações mais detalhadas sobre o funcionamento do script e sua implementação, consulte o código-fonte comentado.
