# hovecraft-SEMEAR2025-grupo2

## BUGS e TO-DO

### no_camera
- Implementar subtração de gaussian blur pra melhorar a nitidez (?)
- Estudar como deixar o código mais leve (adicionar multi-threading. Cuidado pra não pegar coisas de opencl pq o rspi não roda opencl sem umas gambiarras sinistras)
- Implementar buffer
- Remover features que a gente não usa, tipo formato de kernel
- (eu sei que os hz da camera tao capados no nó, mas era so pra alivir um pouco pro meu pc)
- ### TESTAR A PERFORMANCE VS PYTHON

### no_controle
- Tem que implementar uma saida perdido ---> procurando
- Tirar a limitação de hz do controle
- Implementar odometria com a pixhawk (talvez isso acabe sendo outro nó)
- Implmentar uma maneira de diminuir os motores de sustentação nas curvas, pra melhorar o atrito.
- Simular melhor o comportamento do hover no gazebo (alguma maneira de implementar a falta de atrito)
