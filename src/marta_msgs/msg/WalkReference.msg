# Plano do gerador de marcha (OmnidirectionalWalkZMP, 3D-LIPM de Kajita 2001).
# Publicado por controller_node a 100 Hz, logo apos walker.update().
#
# Serve de referencia para comparar com a realidade medida em WalkTelemetry:
# a diferenca entre os dois e o resultado que interessa.

uint8 SUPPORT_DOUBLE = 0
uint8 SUPPORT_LEFT = 1
uint8 SUPPORT_RIGHT = 2

Header header

# Pose planejada do torso. ATENCAO: 'z' aqui e o YAW, nao a altura - e a
# convencao do Vector3 (x, y, theta) usado internamente pelo gerador.
geometry_msgs/Point com

# Pes planejados, RELATIVOS AO TORSO. 'z' de cada um tambem e yaw.
geometry_msgs/Point left_foot
geometry_msgs/Point right_foot

# Altura do torso ate cada pe [m]: z_com no pe de apoio,
# z_com - zFrac*zStep no pe oscilante.
float64 z_left
float64 z_right

float64 phase                 # fase normalizada dentro do passo, [0, 1)
uint8 support_state           # ver constantes acima

# Parametros do LIPM, publicados junto para quem consome nao precisar adivinhar
float64 z_com                 # altura do plano de restricao [m]
float64 t_c                   # constante de tempo sqrt(z_com/g) [s]
float64 step_period           # duracao do passo [s]
float64 double_support_ratio  # fracao do passo em apoio duplo
