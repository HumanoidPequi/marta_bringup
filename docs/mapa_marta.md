# Mapa da Marta

Este documento mostra o mapeamento da comunicação entre os principais nós, tópicos e serviços da simulação da marta

##Nós principais
|  Nó   |  Função |  
|---|---|
| `/ball_tracking` | `recebe imagens da câmera e publica informações relacionadas ao rastreamento da bola` |
| `/servo_angles` | `recebe informações da posição da bola e publica comandos relacionados aos atuadores da cabeça.` |
| `/gazebo` | `Executa a simulação` |
| `/marta/controller_spawner` | `Incializa os controladores da marta` |
| `/robot_state_publisher` | `Recebe os estados das juntas e publica transformações tf.` |
| `/levante_frente` | `Responsavel pela execução do movimento de levantar para frente, publica comandos para as juntas da Marta` |
| `/levante_tras` | `Responsavel pela execução do movimento de levantar tras, publica comandos para as juntas da Marta` |

## Tópicos principais

| Tópico | Tipo de mensagem | Publicador | Assinante | Função |
|---|---|---|---|---|
| `/marta/front_camera/image_raw` | `sensor_msgs/Image` | Gazebo | `/ball_tracking` | Imagem da câmera frontal |
| `/ball_pose` | `std_msgs/Float32MultiArray` | `/ball_tracking` | `/servo_angles` | Posição da bola detectada |
| `/marta/head_pan_position/command` | `std_msgs/Float64` | `/servo_angles` | Gazebo | Comando para movimentar a cabeça horizontalmente |
| `/marta/head_tilt_position/command` | `std_msgs/Float64` | `/servo_angles` | Gazebo | Comando para movimentar a cabeça verticalmente |
|---|---|---|---|---|
| `/marta/front_camera/image_raw` | `sensor_msgs/Image` | Gazebo | `/ball_tracking` | Imagem da câmera frontal |
| `/ball_pose` | `std_msgs/Float32MultiArray` | `/ball_tracking` | `/servo_angles` | Posição da bola detectada |
| `/marta/head_pan_position/command` | `std_msgs/Float64` | `/servo_angles` | Gazebo | Comando para movimentar a cabeça horizontalmente |
| `/marta/head_tilt_position/command` | `std_msgs/Float64` | `/servo_angles` | Gazebo | Comando para movimentar a cabeça verticalmente |

## Serviços principais

| Serviço | Nó que oferece | Tipo  | Função |                                            
|---|---|---|---|
| `/levantar_para_frente`| `/levante_frente` | `marta_msgs/Levantar`| Solicita o movimento de levantar para frente       |
| `/levantar_para_tras`| `/levante_tras`   | `marta_msgs/Levantar`| Solicita o movimento de levantar para trás|
| `/get_angle_x_axis`|`/imu_node`| `std_srvs/Trigger`| Serviço relacionado à obtenção do ângulo no eixo X |
| `/marta/controller_manager/list_controllers` | `/gazebo`| `controller_manager_msgs/ListControllers`  | Lista os controladores gerenciados                 |
| `/marta/controller_manager/switch_controller` | `/gazebo`| `controller_manager_msgs/SwitchController`| Inicia e interrompe controladores                  |




