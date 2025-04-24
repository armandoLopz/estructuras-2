import heapq # Biblioteca para implementar la cola de prioridad (Lista Abierta)

# 1. Distancias por Carretera (Representación del Grafo)

graph = {
    1: {2: 200},
    2: {1: 200, 3: 150, 4: 350, 5: 450},
    3: {2: 150, 5: 400, 6: 225},
    4: {2: 350, 5: 300},    
    5: {2: 450, 3: 400, 4: 300, 7: 250},
    6: {3: 225, 7: 450},
    7: {5: 250, 6: 450, 8: 125},
    8: {7: 125} # La ciudad 8 solo tiene conexión de vuelta a la 7 según la tabla
}

# 2. Distancias Heurísticas (Estimación a la Ciudad 8)

heuristics = {
    1: 800,
    2: 650,
    3: 500,
    4: 650,
    5: 325,
    6: 375,
    7: 125,
    8: 0  # La distancia heurística desde la meta a sí misma es siempre 0
}

# 3. Nodo Inicial y Nodo Meta
start_city = 1
goal_city = 8

print("Datos cargados:")
print(f"Grafo (conexiones y distancias): {graph}")
print(f"Heurísticas (distancia estimada a {goal_city}): {heuristics}")
print(f"Ciudad Inicial: {start_city}")
print(f"Ciudad Meta: {goal_city}")

print("")

def a_star_search(graph, start_node, goal_node, heuristics):
    """
    Implementa el algoritmo A* para encontrar el camino más corto.

    Args:
        graph (dict): El grafo representado como {nodo: {vecino: costo}}.
        start_node: El nodo inicial.
        goal_node: El nodo meta.
        heuristics (dict): Un diccionario con la heurística {nodo: h_costo}.

    Returns:
        tuple: (camino, costo_total) o (None, float('inf')) si no hay camino.
    """
    # Lista Abierta (cola de prioridad): almacena tuplas (f_costo, nodo_actual).
    # El costo f inicial para el nodo de inicio es g(inicio)+h(inicio) = 0 + h(inicio)
    open_list_pq = []
    heapq.heappush(open_list_pq, (heuristics[start_node], start_node))

    # Lista Cerrada: conjunto de nodos ya visitados/expandidos.
    closed_list = set()

    # Diccionario para reconstruir el camino: almacena {nodo: nodo_padre_en_el_camino}.
    came_from = {}

    # Diccionario para almacenar el costo g(n) (costo real desde inicio hasta n).
    # Inicializa todos los costos g como infinito, excepto el nodo inicial (g=0).
    g_costs = {node: float('inf') for node in graph}
    if start_node not in g_costs: # Asegurar que el start_node está en g_costs
         g_costs[start_node] = float('inf')
    g_costs[start_node] = 0


    # Bucle principal del algoritmo A*
    while open_list_pq: # Mientras la lista abierta no esté vacía

        # 1. Extraer el nodo con el menor f_costo de la lista abierta
        # heapq.heappop devuelve la tupla con el menor primer elemento (f_costo)
        current_f, current_node = heapq.heappop(open_list_pq)

        # (Optimización opcional): Si encontramos este nodo antes con un costo f menor,
        # podríamos ignorarlo, pero A* maneja esto correctamente al verificar g_costs más adelante.

        # 2. Comprobar si hemos llegado al nodo meta
        if current_node == goal_node:
            # ¡Éxito! Reconstruir el camino desde la meta hacia atrás usando came_from
            path = []
            temp = current_node
            # Mientras el nodo actual tenga un padre registrado en came_from
            while temp in came_from:
                path.append(temp)
                temp = came_from[temp]
            path.append(start_node) # Añadir el nodo inicial al final
            return path[::-1], g_costs[goal_node] # Devolver el camino invertido y el costo g final

        # 3. Mover el nodo actual a la lista cerrada (ya lo hemos procesado)
        closed_list.add(current_node)

        # 4. Explorar los vecinos del nodo actual
        # Verificar si el nodo actual tiene vecinos definidos en el grafo
        if current_node in graph:
            for neighbor, distance in graph[current_node].items():

                # Ignorar vecinos que ya están en la lista cerrada
                if neighbor in closed_list:
                    continue

                # Calcular el costo g tentativo para ir al vecino a través del nodo actual
                # g(vecino) = g(actual) + costo(actual -> vecino)
                tentative_g = g_costs[current_node] + distance

                # Si este camino hacia el vecino es mejor (más corto) que cualquier
                # camino previamente registrado para él...
                if tentative_g < g_costs.get(neighbor, float('inf')): # Usar .get para manejar nodos no vistos aún
                    # Actualizar la información del vecino:
                    came_from[neighbor] = current_node # Registrar que llegamos al vecino desde el nodo actual
                    g_costs[neighbor] = tentative_g     # Actualizar su costo g
                    # Calcular el costo f del vecino: f = g + h
                    f_cost = tentative_g + heuristics.get(neighbor, float('inf')) # Usar .get por si falta heurística (aunque no debería)
                    # Añadir el vecino a la lista abierta para su futura evaluación
                    heapq.heappush(open_list_pq, (f_cost, neighbor))

    # Si el bucle termina (lista abierta vacía) y no se encontró la meta, no hay camino.
    return None, float('inf')

print("Función a_star_search definida.")

# Ejecutar la búsqueda A* con los datos definidos
print(f"Iniciando búsqueda A* desde {start_city} hasta {goal_city}...")
path, total_cost = a_star_search(graph, start_city, goal_city, heuristics)
print("Búsqueda finalizada.")

# Mostrar los resultados
if path:
    print("-" * 30)
    print("Resultado de la Búsqueda A*:")
    print(f"  Camino encontrado: {' -> '.join(map(str, path))}")
    print(f"  Costo total del camino: {total_cost}")
    print("-" * 30)
else:
    print(f"\nNo se pudo encontrar un camino desde la ciudad {start_city} hasta la ciudad {goal_city}.")