# Simulador de drone
Esse projeto faz parte da etapa de treinamento da equipe Black Bee Drones. É um simulador de drone simples.

## Instalar
O projeto foi criado usando ROS2. Para baixar as dependências do projeto, execute:
`pip install -r requirements.txt`

## Executar
Para executar é necessários executar os comandos a seguir em terminais diferentes:
* `py view_node.py` -> Inicia o nó para visualizar os status do drone;
* `py joystick_node.py` -> Inicia o nó para controlar o drone;
* `py drone_node.py` -> Inicia o nó do drone.
