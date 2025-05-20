import numpy as np
import math

NUMBER_OF_JOINTS = 4

def sysCall_init():
    sim = require('sim')
    self.q = np.zeros(NUMBER_OF_JOINTS)
    # Handles das juntas
    self.joints_hdl = [
        sim.getObject("../base_link_to_link1"),
        sim.getObject("../link1_to_link2"),
        sim.getObject("../link2_to_link3"),
        sim.getObject("../link3_to_gripper_link")
    ]
    self.gripper_link = sim.getObject("../link3_to_hand_tcp")
    
    # Parâmetros de desenho
    self.drawing = True
    self.line_handles = []
    self.start_time = sim.getSimulationTime()
    self.stabilize_time = 3.0
    self.total_duration = 20.0  # tempo total aumentado para movimento mais controlado
    self.initial_joint_positions = [0.0, 0.0, -0.8, 0.0]
    
    # Configura cada junta para modo posição com força máxima
    for i, jh in enumerate(self.joints_hdl):
        sim.setJointMode(jh, sim.jointmode_force, 0)
        sim.setJointMaxForce(jh, 100)  # adapte se necessário
        sim.setJointPosition(jh, self.initial_joint_positions[i])
        sim.setJointTargetPosition(jh, self.initial_joint_positions[i])
    
    self.phase = "stabilize"
    
    # Configura pontos de controle para desenhar "IC"
    # Definindo os waypoints para desenhar I e C
    self.waypoints = [
        # Letra I - linha vertical maior
        {"joint_angles": [0.0, 0.0, -0.8, 0.0], "progress": 0.0},  # Ponto inicial
        {"joint_angles": [0.0, 0.15, -1.0, 0.0], "progress": 0.1},  # Topo do I
        {"joint_angles": [0.0, -0.15, -0.65, 0.0], "progress": 0.2},  # Base do I
        
        # Mover para posição inicial do C (levantar o braço)
        {"joint_angles": [0.0, 0.0, -0.8, 0.0], "progress": 0.25, "pen_up": True},
        
        # Reposicionar para começar o C (à direita do I, abertura virada para o I)
        {"joint_angles": [-0.25, 0.15, -0.95, 0.0], "progress": 0.3, "pen_up": True},
        
        # Letra C - com abertura à esquerda (em direção ao I)
        {"joint_angles": [-0.25, 0.15, -0.95, 0.0], "progress": 0.35},  # Início do topo do C
        {"joint_angles": [-0.2, 0.12, -0.92, 0.0], "progress": 0.4},  # Curva superior do C
        {"joint_angles": [-0.15, 0.08, -0.88, 0.0], "progress": 0.45},  # Seguindo para a esquerda
        {"joint_angles": [-0.13, 0.04, -0.85, 0.0], "progress": 0.5},  # Ponto mais à esquerda (próximo ao I)
        {"joint_angles": [-0.13, 0.0, -0.82, 0.0], "progress": 0.55},  # Meio do C
        {"joint_angles": [-0.13, -0.04, -0.78, 0.0], "progress": 0.6},  # Continuando curva
        {"joint_angles": [-0.15, -0.08, -0.75, 0.0], "progress": 0.65},  # Iniciando base
        {"joint_angles": [-0.2, -0.12, -0.72, 0.0], "progress": 0.7},  # Curva inferior
        {"joint_angles": [-0.25, -0.15, -0.7, 0.0], "progress": 0.75},  # Base final do C
        
       
    ]
    
    print(">> sysCall_init: Posição inicial aplicada; esperando estabilizar.")

def interpolate_joint_angles(current_progress, waypoints):
    """Interpola os ângulos das juntas entre waypoints"""
    # Encontra os dois waypoints entre os quais estamos
    wp1 = None
    wp2 = None
    for i, wp in enumerate(waypoints):
        if wp["progress"] >= current_progress:
            if i > 0:
                wp1 = waypoints[i-1]
                wp2 = wp
            else:
                # Se estamos antes do primeiro waypoint, use o primeiro
                return waypoints[0]["joint_angles"], waypoints[0].get("pen_up", False)
            break
    
    if wp1 is None or wp2 is None:
        # Se estivermos depois do último waypoint, use o último
        return waypoints[-1]["joint_angles"], waypoints[-1].get("pen_up", False)
    
    # Calcula a interpolação linear entre os dois waypoints
    p1 = wp1["progress"]
    p2 = wp2["progress"]
    
    # Normaliza o progresso entre os dois waypoints (0 a 1)
    t = (current_progress - p1) / (p2 - p1) if p2 > p1 else 0
    
    # Interpola cada ângulo de junta
    result = []
    for i in range(len(wp1["joint_angles"])):
        angle1 = wp1["joint_angles"][i]
        angle2 = wp2["joint_angles"][i]
        result.append(angle1 + t * (angle2 - angle1))
    
    # Verifica se é um movimento com a caneta levantada
    pen_up = wp2.get("pen_up", False)
    
    return result, pen_up

def sysCall_actuation():
    sim = require('sim')
    t = sim.getSimulationTime()
    elapsed = t - self.start_time
    
    if self.phase == "stabilize":
        # mantém posição inicial
        for i, jh in enumerate(self.joints_hdl):
            sim.setJointTargetPosition(jh, self.initial_joint_positions[i])
        
        if elapsed >= self.stabilize_time:
            self.phase = "drawing"
            self.draw_start_time = t
            self.start_pos = sim.getObjectPosition(self.gripper_link, -1)
            self.last_pos = list(self.start_pos)
            self.pen_up = False
            print(f">> Iniciando desenho das letras IC; TCP inicial = {self.start_pos}")
    
    elif self.phase == "drawing":
        draw_elapsed = t - self.draw_start_time
        drawing_duration = self.total_duration - self.stabilize_time
        progress = min(draw_elapsed / drawing_duration, 1.0)
        
        # Interpola os ângulos das juntas baseados no progresso atual
        qd, pen_up = interpolate_joint_angles(progress, self.waypoints)
        
        # Aplica os alvos de posição
        for i, jh in enumerate(self.joints_hdl):
            sim.setJointTargetPosition(jh, qd[i])
        
        # Verifica a altura do TCP
        grip = sim.getObjectPosition(self.gripper_link, -1)
        if grip[2] < 0.1:
            print("!! ALERTA: TCP abaixo de 0.1m; ajustando emergência")
            # Sobe levemente juntas 1 e 2
            sim.setJointTargetPosition(self.joints_hdl[1], qd[1] + 0.1)
            sim.setJointTargetPosition(self.joints_hdl[2], qd[2] - 0.05)
        
        # Desenha segmento se a caneta estiver abaixada
        if not pen_up and not self.pen_up:  # Apenas desenhe se a caneta estiver abaixada
            line = sim.addDrawingObject(sim.drawing_lines, 3, 10, -1, 0, [0,0,1])  # Linha mais grossa
            sim.addDrawingObjectItem(line, [*self.last_pos, *grip])
            self.line_handles.append(line)
        
        self.pen_up = pen_up
        self.last_pos = list(grip)
        
        if progress >= 1.0:
            self.phase = "finished"
            dx, dy, dz = grip[0] - self.start_pos[0], grip[1] - self.start_pos[1], grip[2] - self.start_pos[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            print(f">> Desenho concluído! Comprimento = {dist:.3f}m")
    
    # fase "finished" mantém parada automática

def sysCall_sensing():
    sim = require('sim')
    if self.phase == "drawing":
        grip = sim.getObjectPosition(self.gripper_link, -1)
        if grip[2] < 0.1:
            print(f"ATENÇÃO: TCP muito baixo (Z = {grip[2]:.4f}m)")

def sysCall_cleanup():
    sim = require('sim')
    for h in self.line_handles:
        sim.removeDrawingObject(h)
    
    safe = [0.0, 1.2, -0.8, 0.0]
    for i, jh in enumerate(self.joints_hdl):
        sim.setJointTargetPosition(jh, safe[i])
    
    print(">> sysCall_cleanup: Limpeza e retorno à posição segura.")
