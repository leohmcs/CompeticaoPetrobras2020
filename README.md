Instruções de instalação:
  - Siga o processo de instalação do https://github.com/ctu-mrs/mrs_uav_system ( Na aba de Installation );
  - No diretório do mrs_workspace/src;
  - Faça o git clone desse pacote;
  - Execute um catkin build  
  - Siga as instruções do video para configurar as fases:
  
    https://youtu.be/aM0vAr_YDT8
    
  - Pelo terminal acesse o diretório da pasta start, presente nesse pacote;
  - Execute o comando no terminal:
  
    $ ./start.sh

Apos isso a simulação deve iniciar, com o drone na origem.
- Para alterar a posição de spawn do drone acesse o arquivo uav1_pos.txt na pasta start
- Sensores não disponiveis:

  --enable-magnetic-gripper
  
  --enable-mobius-camera-back-left
  
  --enable-mobius-camera-back-right
  
  --enable-ouster
  
  --enable-pendulum
  
  --enable-realsense-top
  
  --enable-teraranger
  
  --enable-uv-camera
  
  --enable-uv-leds
  
  --enable-uv-leds-beacon
  
  --enable-whycon-box
  
