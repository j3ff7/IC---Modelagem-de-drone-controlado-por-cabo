# 🚁 Modelagem de Drone Controlado por Cabo (Iniciação Científica)

Este repositório contém os códigos, modelos e ambientes de simulação desenvolvidos para o projeto de Iniciação Científica (IC). O foco principal é a modelagem matemática, controle e simulação da dinâmica de um drone acoplado a um cabo (tethered drone) e seu respectivo sistema de guincho.

⚠️ **Atenção:** Todos os códigos-fonte atualizados e pacotes principais estão localizados na pasta **`src/`**. 

## 📂 Estrutura do Repositório

A organização atual do projeto está distribuída da seguinte forma:

* **`src/`**: Diretório principal. Contém os códigos mais recentes, pacotes ROS2, scripts de controle e plugins desenvolvidos (como o plugin do guincho).
* **`Gazebo/`**: Contém os arquivos descritivos e modelos físicos do sistema, incluindo o arquivo de modelo do cabo flexível utilizado na simulação.
* *(Outros diretórios)*: Podem conter documentações de apoio, relatórios em LaTeX, rascunhos em MATLAB ou versões legadas dos scripts.

## 🛠️ Tecnologias Utilizadas

O ambiente de simulação e controle foi construído para operar em ambiente Linux (Ubuntu) utilizando as seguintes ferramentas:

* **ROS2:** Middleware para comunicação e arquitetura de controle do drone.
* **Gazebo:** Simulador físico responsável por renderizar a dinâmica de voo, a física do cabo flexível e a atuação do guincho.
* **C++ / Python:** Linguagens base para a criação dos nós e plugins da simulação.

## 🚀 Como Compilar e Executar

1. Clone este repositório para o seu workspace local:
   ```bash
   git clone [https://github.com/j3ff7/IC---Modelagem-de-drone-controlado-por-cabo.git](https://github.com/j3ff7/IC---Modelagem-de-drone-controlado-por-cabo.git)
