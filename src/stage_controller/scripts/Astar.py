#!/usr/bin/env python3

import heapq

def astar(start, goal, neighbors, heuristic):
    """
    Implementação do algoritmo A* em Python.

    Args:
        start: O nó inicial do grafo.
        goal: O nó objetivo a ser alcançado.
        neighbors: Uma função que recebe um nó e retorna seus vizinhos.
        heuristic: Uma função heurística que estima o custo de alcançar o objetivo a partir de um determinado nó.

    Returns:
        O caminho do nó inicial ao nó objetivo como uma lista de nós.
    """

    # Inicialização dos conjuntos abertos e fechados
    open_set = [(0, start)]  # Fila de prioridade dos nós a serem explorados, ordenados pelo custo estimado (f = g + h)
    closed_set = set()  # Conjunto de nós já explorados

    # Dicionários para rastrear os custos
    g_scores = {start: 0}  # Custo atual para chegar a um determinado nó
    f_scores = {start: heuristic(start, goal)}  # Estimativa do custo total para alcançar o objetivo passando por um determinado nó

    while open_set:
        # Seleciona o nó com o menor custo estimado (f score)
        current_cost, current_node = heapq.heappop(open_set)

        # Verifica se alcançamos o objetivo
        if current_node == goal:
            # Reconstrói o caminho percorrendo os pais a partir do nó objetivo
            path = [current_node]
            while current_node in neighbors:
                current_node = neighbors[current_node]
                path.append(current_node)
            return path[::-1]  # Retorna o caminho em ordem correta

        # Adiciona o nó atual ao conjunto fechado
        closed_set.add(current_node)

        # Explora os vizinhos do nó atual
        for neighbor in neighbors(current_node):
            if neighbor in closed_set:
                continue  # Ignora nós já explorados

            # Calcula o custo do caminho atual para alcançar o vizinho
            g_score = g_scores[current_node] + heuristic(current_node, neighbor)

            if neighbor not in g_scores or g_score < g_scores[neighbor]:
                # Atualiza o custo do caminho para o vizinho, se necessário
                g_scores[neighbor] = g_score
                f_score = g_score + heuristic(neighbor, goal)
                f_scores[neighbor] = f_score

                # Adiciona o vizinho ao conjunto aberto, se ainda não estiver presente
                heapq.heappush(open_set, (f_score, neighbor))

    # Se não for possível encontrar um caminho, retorna None
    return None