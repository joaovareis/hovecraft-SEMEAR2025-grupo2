# hovecraft-SEMEAR2025-grupo2

## BUGS e TO-DO

### no_camera
- Implementar subtração de gaussian blur pra melhorar a nitidez (?)
- Estudar como deixar o código mais leve (adicionar multi-threading. Cuidado pra não pegar coisas de opencl pq o rspi não roda opencl sem umas gambiarras sinistras)
- Implementar buffer
- Checar se contours ta fornecendo o centro adequado do objeto
- Remover features que a gente não usa, tipo formato de kernel
- (eu sei que os hz da camera tao capados no nó, mas era so pra alivir um pouco pro meu pc)
- ### TESTAR A PERFORMANCE VS PYTHON

### no_controle
- Atualmente o hover vira demais pra esquerda quando o objeto está bem longe, fazendo ele sair da vista, resultando num comportamento de andar em meia-luas até um certo tamanho de objeto
- O hover está descentralizando a camera depois que implementei a camera em c++. Eu, joão, acho que é o offset, mas talvez tenha que mexer no PID
- Tem que implementar uma saida perdido ---> procurando
- Tornar a velocidade gradual com a área, basicamente regra de tres
- Tirar a limitação de hz do controle
- Implementar odometria com a pixhawk (talvez isso acabe sendo outro nó)
- Implmentar uma maneira de diminuir os motores de sustentação nas curvas, pra melhorar o atrito.
- Simular melhor o comportamento do hover no gazebo (alguma maneira de implementar a falta de atrito)
