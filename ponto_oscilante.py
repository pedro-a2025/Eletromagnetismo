#importando as bibliotecas
import numpy as np
#import manim
from manim import *

#Equação utilizada: livro do jackson Jackson de eletromagnetismo, número da equação: 14.14
class PontoOscilante(Scene):
    def construct(self):

        
        #configurando o tamanho da camera
        self.camera.frame_width = 28  # Set width to 10 units
        self.camera.frame_height = 16  # Set height to 5 units

        v = 0.996 #velocidade utilizada para o movimento do ponto
        def trajectory(t): #função trejetória do ponto
            #Uso da função heaviside para que f(t) = 0 para todo t < 0 e f(t) = 1 para todo t >= 0
            a = np.array([np.heaviside(t, 0)*0.3*t, np.heaviside(t, 0)*0.3*np.sin(t), 0]) #x = 0.3t e y = 0.3*sen(t)
            b = np.array([np.heaviside(t, 0)*0.3*np.cos(t), np.heaviside(t, 0)*0.3*np.sin(t), 0])  #x = 0.3cos(t) e y = 0.3sen(t)
            return a #nesse caso especifico, será feita uma trejetória senoidal, mas poderia ter sido a circular ou qualquer outra definida da msm forma

        def velocity(t): #função velocidade do ponto, derivada da trejetória
            a = np.array([np.heaviside(t, 0)*0.3, np.heaviside(t, 0)*0.3*np.cos(t), 0])
            b = np.array([-np.heaviside(t, 0)*0.3*np.sin(t), np.heaviside(t, 0)*0.3*np.cos(t), 0])
            return a

        def acceleration(t): #função aceleração do ponto, derivada da aceleração
            a = np.array([np.heaviside(t, 0)*0, -np.heaviside(t, 0)*0.3*np.sin(t), 0])
            b = np.array([-np.heaviside(t, 0)*0.3*np.cos(t), -np.heaviside(t, 0)*0.3*np.sin(t), 0])
            return a

        def electric_field(r_obs, t): #definindo a função do campo elétrico na forma vetorial
            def tempo_de_retardo(): #tempo desde que há o movimento da bolinha até a sua consequência no campo
                max_iter=100 #n° interação arbitrário, quanto maior, mais fiel a realidade, porém há mais processamento
                t_r = t
                for i in range(max_iter):
                    r_q = trajectory(t_r)
                    R = np.linalg.norm(r_obs - r_q)
                    t_r_new = t - R
                    if abs(t_r_new - t_r) < 1e-6: #definindo condições de existência, evitando divisões por zero
                        break
                    t_r = t_r_new
                return t_r

            # Aplicando o tempo de retardo nas funções pré definidas
            t_r = tempo_de_retardo()
            r_q = trajectory(t_r)
            v_q = velocity(t_r)
            a_q = acceleration(t_r)
            beta = np.linalg.norm(v_q)

            # Calculando vetor deslocamento (R)
            R_vec = r_obs - r_q
            R_mag = np.linalg.norm(R_vec)
            if R_mag < 1e-30:
                R_mag = 1e-30 #condição de existência, evitando dividir por zero
            R_hat = R_vec / R_mag

            Ev = (R_hat - v_q)*(1 - beta**2)/(R_mag**2*(1 - np.dot(R_hat, v_q))**3) #parte relacionada a velocidade
            Ea = np.cross(R_hat, np.cross((R_hat - v_q), a_q))/(R_mag*(1 - np.dot(R_hat, v_q))**3) #parte relacionada a aceleração
            return Ev + Ea

        def electric_field_vec(pos, t): #retorna o campo na sua forma vetorial
            return np.array([electric_field(pos, t)[0], electric_field(pos, t)[1], 0]) 

        # Mostra o tempo na tela
        tempo_texto = Text("Tempo: 0.00").to_corner(UL)

        def atualizar_tempo(): 
            tempo_de_animacao = self.renderer.time - waiting_time
            if tempo_de_animacao <= 0: #inicia o movimento com tempo = 0
                tempo_de_animacao = 0
            return Text(f"Tempo: {tempo_de_animacao:.2f}").to_corner(UL)

        tempo_texto.add_updater(lambda mob, dt: mob.become(atualizar_tempo()))
        #self.add(tempo_texto)

        ponto = Dot(point=trajectory(self.renderer.time), radius=0.1, color=RED) #define a trajetória do ponto com o tempo atual da animação
        vector_field_scale = 1.4 #escala do campo vetorial
        grid = [-12,12,0.5] #tamanho do campo vetorial [x,y, escala]
        vector_field = ArrowVectorField(lambda pos: vector_field_scale*electric_field_vec(pos, 0.0), x_range=grid, y_range=grid) #campo vetorial
        tstart = self.renderer.time #
        #self.play(Create(ponto))
        self.add(ponto, tempo_texto, vector_field)
        tend = self.renderer.time #
        deltat = tend - tstart #calcula o tempo da adição dos objetos no video
        #self.add(vector_field)
        waiting_time = 1 #tempo de espera para iniciar
        self.wait(waiting_time) 
        
        

        def update_field(mob, dt):  #função de atualização do campo vetorial
            #velocidade_oscilacao = 3  # Ajuste este valor para controlar a velocidade
            tempo_inicial = waiting_time #- deltat
            tempo_atual = self.renderer.time
            tempo_utilizado = tempo_atual - tempo_inicial
            mob.become(ArrowVectorField(lambda pos: vector_field_scale*electric_field_vec(pos, tempo_utilizado), x_range=grid, y_range=grid))
        def atualizar_posicao_ponto(ponto): #funlão de atualização do campo
            #velocidade_oscilacao = 3  # Ajuste este valor para controlar a velocidade
           #delay = 12 #calculado manualmente em uma parte da realização do código, mas não útil posteriormente graças a alguns ajustes
            tempo_inicial = waiting_time #+ deltat
            tempo_atual = self.renderer.time
            tempo_utilizado = tempo_atual - tempo_inicial #- delay
            ponto.move_to(trajectory((tempo_utilizado)))
            #ponto.move_to(trajectory(self.renderer.time - waiting_time - deltat))

        
        vector_field.add_updater(update_field) #aplica a função de atualização do campo vetorial no campo vetorial
        #self.add(vector_field)
        #self.wait(10)
        ponto.add_updater(atualizar_posicao_ponto) #aplica a função de atualização do ponto no ponto
        self.wait(1) #Tempo de oscilação
