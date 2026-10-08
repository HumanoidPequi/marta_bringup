# Estado MEDIDO da marcha e as grandezas do 3D-LIPM derivadas dele.
# Publicado por walk_telemetry_node a 100 Hz.
#
# O CoM vem da verdade de solo do Gazebo (soma massa x posicao sobre os links),
# nao de estimativa embarcada. Serve para VALIDAR o controlador com uma regua
# independente do sinal que ele proprio regula.
#
# Referencias de equacao sao de Kajita et al., "The 3D Linear Inverted Pendulum
# Mode", IROS 2001.
#
# NENHUM ARRAY nesta mensagem: o PlotJuggler tem bug conhecido (issue #224) em
# que campos declarados depois de um array nao aparecem para plotagem. Se algum
# array for acrescentado no futuro, deve ir no fim.

uint8 SUPPORT_DOUBLE = 0
uint8 SUPPORT_LEFT = 1
uint8 SUPPORT_RIGHT = 2

Header header

# ---- realidade, no frame do mundo ----
geometry_msgs/Point   com           # centro de massa [m]
geometry_msgs/Vector3 com_vel       # velocidade do CoM [m/s]
geometry_msgs/Point   support_foot  # pe de apoio [m]
geometry_msgs/Point   swing_foot    # pe oscilante [m]

# ---- realidade, no frame do PE DE APOIO ----
# E o frame do paper: a origem esta no pe que sustenta, e por isso as curvas
# da figura 5 saltam a cada troca de apoio.
geometry_msgs/Point   com_support
geometry_msgs/Vector3 com_vel_support

# ---- referencia no mesmo frame, e o erro ----
geometry_msgs/Point   com_ref_support
geometry_msgs/Vector3 com_error     # referencia menos medido [m]

# ---- grandezas do 3D-LIPM ----
float64 orbital_energy_x   # eq. (29): -(g/2zc)x^2 + (1/2)xdot^2. Invariante sob
float64 orbital_energy_y   # eq. (28): o LIPM; deve ficar constante em apoio simples.
float64 dcm_x              # ponto de captura x + t_c*xdot [m] (extensao moderna)
float64 dcm_y
float64 zmp_est_x          # x - (zc/g)*xddot [m]. ESTIMADO do CoM: nao ha sensor
float64 zmp_est_y          # de forca nos pes, e isso ignora momento angular.

# ---- atitude do tronco ----
float64 pitch          # estimada pelo torso_attitude_node [rad]
float64 pitch_rate
float64 roll
float64 roll_rate
float64 pitch_true     # verdade de solo do Gazebo [rad]
float64 roll_true

# ---- estado da marcha ----
float64 phase
uint8   support_state
float64 height         # altura do tronco [m]; arbitro final de queda
float64 total_mass       # massa somada dos links [kg]; sanidade do calculo do CoM

# Altura MEDIDA do CoM acima do ponto de contato do pe de apoio.
# Nao confundir com o z_com de WalkReference: aquele e o parametro que o gerador
# assume (distancia torso-pe da IK, 0.2975 m), este e o que o robo realmente tem
# (~0.218 m). A diferenca da ~36% em g/zc, que e o UNICO parametro do LIPM.
# Publicado para que a analise possa recalcular as energias com qualquer um dos
# dois e quantificar esse descasamento de modelo.
float64 z_com_measured

# ---- yaw: verdade de solo do Gazebo (investigacao do desvio de direcao) ----
# Nao ha estimativa embarcada de yaw nem controle nesse eixo; estes campos sao
# a regua. Yaw ZYX em [-pi, pi] (a analise desenrola); taxas vem do twist de
# cada link, sem derivada numerica.
# ATENCAO: as taxas NAO sao integraveis a 100 Hz. No impacto do pe elas tem
# picos de +-190 graus/s que a fisica de 1 kHz resolve e esta amostragem nao
# (aliasing: integrar a do pe esquerdo deu -269 graus contra +89 do angulo).
# Para giro acumulado, use sempre diferenca de ANGULO.
float64 yaw_true             # base_link [rad]
float64 yaw_rate_true        # [rad/s]
float64 left_foot_yaw        # l_feet_1 [rad]
float64 left_foot_yaw_rate   # [rad/s]; em apoio, != 0 significa o pe girando no chao
float64 right_foot_yaw
float64 right_foot_yaw_rate

# Ponto de contato de CADA pe, no mundo, independente do estado de apoio
# (support_foot/swing_foot dependem do plano e viram o ponto medio em apoio
# duplo). E o que permite detectar toque e saida pela altura do pe.
geometry_msgs/Point left_foot_contact
geometry_msgs/Point right_foot_contact
