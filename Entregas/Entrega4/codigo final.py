# -*- coding: utf-8 -*-

###############################################################################
# Sección 0: Importaciones.
###############################################################################

# Para optimizar.
import gurobipy as gp

# Para leer los datos.
import pandas as pd
from io import StringIO

#pip install matplotlib==3.5.1
#pip install --update networkx

# Para visualizar el grafo y la solución.
import networkx as nx
import matplotlib.pyplot as plt

# Para calcular la demora computacional.
import time


###############################################################################
# Sección 1: Definición de los parámetros.
###############################################################################

# Guardar el tiempo de inicio.
inicio = time.time()

# Crear el grafo
G = nx.Graph()

# Cantidad de nodos.
cantidad_nodos = 14

# Definir los datos del sistema de 14 barras.
nref = 14          # Nodo de referencia
vref = 1.0         # Tensión en la subestación (pu)
vbase = 23.0       # Tensión base (kV)
sbase = 100000     # Potencia base (kVA) potencia aparente base
tol = 1e-8         # Tolerancia del error permitido
vmin = 0.93        # Tensión mínima (pu)
vmax = 1.05        # Tensión máxima (pu)

# Base de impedancia.
zbase = 100


###############################################################################
# Sección 2: lectura de datos.
###############################################################################

# La cadena de texto proporcionada
cadena_texto = """14      13       7.50    10.00
              13      12       8.00    11.00
              13      11       9.00    18.00
              11      10       4.00     4.00
              14       9      11.00    11.00
               9       8       8.00    11.00
               9       7      11.00    11.00
               8       6      11.00    11.00
               8       5       8.00    11.00
              14       4      11.00    11.00
               4       3       9.00    12.00
               4       2       8.00    11.00
               2       1       4.00     4.00
              12       6       4.00     4.00
               7       3       4.00     4.00
              10       1       9.00    12.00"""
              
# Leer la cadena de texto.
cadena_data = StringIO(cadena_texto)
df_data = pd.read_csv(cadena_data, delim_whitespace=True, header=None)

# Crear diccionarios R y X.
R = {}
X = {}

# Iterar a través de cada fila en el DataFrame.
for index, row in df_data.iterrows():
    # Extraer los valores de las columnas.
    nodo_inicio = int(row[0])
    nodo_fin = int(row[1])
    valor_R = float(row[2])
    valor_X = float(row[3])

    # Crear las tuplas para usar como claves en los diccionarios.
    clave_directa = (nodo_inicio, nodo_fin)
    clave_inversa = (nodo_fin, nodo_inicio)

    # Agregar las tuplas como claves en los diccionarios R y X.
    R[clave_directa] = valor_R
    R[clave_inversa] = valor_R  # También agregar la dirección inversa
    
    X[clave_directa] = valor_X
    X[clave_inversa] = valor_X  # También agregar la dirección inversa


# Guardar los datos en un string.
data_str = """
              14          0.0     0.0     0.0
              13       2000.0  1600.0     0.0
              12       3000.0  1500.0  1100.0
              11       2000.0   800.0  1200.0
              10       1500.0  1200.0     0.0
               9       4000.0  2700.0     0.0
               8       5000.0  3000.0  1200.0
               7       1000.0   900.0     0.0
               6        600.0   100.0   600.0
               5       4500.0  2000.0  3700.0
               4       1000.0   900.0     0.0
               3       1000.0   700.0  1800.0
               2       1000.0   900.0     0.0
               1       2100.0  1000.0  1800.0
"""

# Leer la cadena.
data = StringIO(data_str)

# Lee los datos en un DataFrame.
df = pd.read_csv(data, delim_whitespace=True, header=None)

# Crea un diccionario para almacenar los valores.
barras = {}

# Itera a través de cada fila en el DataFrame.
for index, row in df.iterrows():
    # Extraer los valores de "barra", "pd", y "qd".
    barra = int(row[0])
    pd_value = row[1]
    qd_value = row[2]
    
    # Crear una lista con los valores de pd y qd.
    values_list = [pd_value, qd_value]
    
    # Agregar la lista de valores al diccionario con "barra" como clave.
    barras[barra] = values_list


###############################################################################
# Sección 3: Creación de las estructuras de datos.
###############################################################################

# Definir el conjunto de nodos.
Nodos = list(range(1, cantidad_nodos + 1))

# Definir le conjunto de períodos de tiempo.
#T = range(1, 14)
T=range(1,2)
G.add_nodes_from(Nodos)

for nodo in range(0,cantidad_nodos):
    G.nodes[cantidad_nodos-nodo]['Pd'] = barras[cantidad_nodos-nodo][0]  # Asigna Pd
    G.nodes[cantidad_nodos-nodo]['Qd'] = barras[cantidad_nodos-nodo][1]  # Asigna Qd

# Define los arcos y asigna características desde los diccionarios R y Reactancia
for (arco, r), (arco, x) in zip(R.items(), X.items()):
    G.add_edge(arco[0], arco[1], resistencia=r, reactancia=x)
    

# Obtén todos los arcos del grafo G, considerando ambos sentidos
Arcos = set(G.edges()).union(set([(arco[1], arco[0]) for arco in G.edges()]))


###############################################################################
# Sección 4: Visualización del grafo inicial
###############################################################################
import matplotlib
print(matplotlib.__version__)

pos = nx.spring_layout(G)
edge_labels = {(i, j): f"R:{int(G[i][j]['resistencia'])}\nX:{int(G[i][j]['reactancia'])}" for i, j in G.edges()}
nx.draw(G, pos, with_labels=True, node_size=300, node_color='lightgray',  width=1, font_size=15)
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
plt.title("Grafo Inicial", size=10)
plt.show()

###############################################################################
# Sección 5: Definir el modelo de optimización.
###############################################################################

# Crear el modelo.
model = gp.Model('RSDEE')

# Variables de decisión: x[r, j] será 1 si el recurso 'r' es asignado al trabajo 'j', y 0 de lo contrario.
P = model.addVars( Arcos, T, vtype=gp.GRB.CONTINUOUS, name="P") # flujo de potencia activa en el arco ij en el per´ıodo t
Ps = model.addVars( Nodos, T, vtype=gp.GRB.CONTINUOUS, name="Ps") # flujo de potencia activa de subestación
Q = model.addVars( Arcos, T, vtype=gp.GRB.CONTINUOUS, name="q") #flujo de potencia reactiva en el arco ij en el per´ıodo t.
Qs = model.addVars( Nodos, T, vtype=gp.GRB.CONTINUOUS, name="Qs") # flujo de potencia reactiva de subestación
I_sqr = model.addVars( Arcos, T, vtype=gp.GRB.CONTINUOUS, name="i_sqr") # la ra´ız del flujo de corriente en el arco ij en el per´ıodo t.
V_sqr = model.addVars(Nodos, T, vtype=gp.GRB.CONTINUOUS, name="v_sqr") # la ra´ız del voltaje presente en el nodo i en el per´ıodo t.
Delta_V = model.addVars(Arcos, T, vtype=gp.GRB.CONTINUOUS, lb=0, name='Delta_Discretización') # intervalo de discretizaci´on de V
y = model.addVars(Arcos, T, vtype=gp.GRB.BINARY, name="y") #define si el interruptor en el arco ij estar´a activo en el per´ıodo t.

# Definir la función objetivo.
model.setObjective(gp.quicksum(R.get((i,j),0) * I_sqr[i, j, t] for (i,j) in Arcos for t in T ), gp.GRB.MINIMIZE)


# -----------------------------------------------------------------------------
# Definición de las restricciones.
# -----------------------------------------------------------------------------

# Ecuación de balance de potencia activa.
for i in Nodos:
  for t in T:
        model.addConstr(gp.quicksum(P[k,i, t] for (k,i) in Arcos)- gp.quicksum(P[i, j, t]+ R.get((i,j),0)*I_sqr[i,j,t] for (i,j) in Arcos)+ Ps[i,t] == G.nodes[i]["Pd"], name=f"balance de potencia activa_{i}_{t}")

# Ecuación de balance de potencia reactiva.
for i in Nodos:
  for t in T: 
        model.addConstr(gp.quicksum(Q[k,i, t] for (k,i) in Arcos)- gp.quicksum(Q[i, j, t]+ X.get((i,j),0)*I_sqr[i,j,t] for (i,j) in Arcos)+ Qs[i,t] == G.nodes[i]["Qd"], name=f"balance de potencia reactiva{i}_{t}")

# La disminución del voltaje en un arco particular.
for (i,j) in Arcos:
  Z2 = (R.get((i,j),0))**2+(X.get((i,j),0))**2
  for t in T: 
        model.addConstr(V_sqr[i,t]==V_sqr[j,t]+2*(R.get((i,j),0)*P[i,j,t]+X.get((i,j),0)*Q[i,j,t])-Z2*I_sqr[i,j,t]+Delta_V[i,j,t], name=f"disminucion voltaje_{i}_{j}_{t}")

# La disminuci´on del voltaje en un arco particular.
for (i, j) in Arcos:
    for t in T:
        bV = 0.1
        model.addConstr(-bV * (1 - y[i, j, t]) <= Delta_V[i, j, t], name=f"lower_bound_{i}_{j}_{t}")
        model.addConstr(Delta_V[i, j, t] <= bV * (1 - y[i, j, t]), name=f"upper_bound_{i}_{j}_{t}")

# Calcular la magnitud del flujo de corriente de un arco particular.
for (i,j) in Arcos:
  for t in T:         
        model.addConstr(V_sqr[i,t]*I_sqr[i,j,t]>=P[i,j,t]**2+Q[i,j, t]**2, name=f"intervalo_{(i,j)}_{t}")

# Definir los l´ımites del voltaje.

for i in Nodos:
    for t in T:
        model.addConstr(V_sqr[i, t] >= vmin**2, name=f"lower_bound_{i}_{t}")
        model.addConstr(V_sqr[i, t] <= vmax**2, name=f"upper_bound_{i}_{t}")

# Definir los límites del flujo de corriente
for (i,j) in Arcos:
  for t in T: 
      I_max = 1
      model.addConstr(0 <= I_sqr[i, j, t], name=f"lower_bound_{i}_{j}_{t}")
      model.addConstr(I_sqr[i, j, t] <= (I_max**2) * y[i, j, t], name=f"upper_bound_{i}_{j}_{t}")

# Topología radial.
for t in T:
  model.addConstr(gp.quicksum(y[i,j,t] for (i,j) in Arcos )==cantidad_nodos -1, name="topología radial")

# Restricción para tener solo un sentido:
for t in T:
    for (i,j) in Arcos:
        model.addConstr(y[i,j,t] +y[j,i,t] <=1, name="arco unitario")

# -----------------------------------------------------------------------------
# Solución.
# -----------------------------------------------------------------------------

# Guardar el modelo para inspección posterior.
model.write('RSDEE.lp')

# Ejecutar el optimizador.
model.optimize()

# Comprobar si el modelo se ha resuelto de manera óptima.
if model.status == gp.GRB.OPTIMAL:
    print("La solución óptima se ha encontrado.")
    print("Valor de la función objetivo:", model.objVal)

    # Imprimir valores de las variables y[i, j, t] cuando son iguales a 1.
    for (i, j, t) in y:
        if y[i, j, t].x == 1:
            print(f"y[{i}, {j}] = {y[i, j, t].x}")

else:
    print("No se encontró una solución óptima.")    

# Guardar el tiempo de finalización.
fin = time.time()

# Calcular el tiempo total en milisegundos y segundos.
tiempo_total_milisegundos = (fin - inicio) * 1000
tiempo_total_segundos = fin - inicio

print(f"El procedimiento tomó {tiempo_total_milisegundos:.2f} milisegundos ({tiempo_total_segundos:.2f} segundos).")


###############################################################################
# Sección 5: visualización de la solución.
###############################################################################

# Crear un grafo dirigido vacío.
G_solution = nx.DiGraph()

# Añadir los nodos al grafo.
G_solution.add_nodes_from(Nodos)

# Añadir las aristas que pertenecen a la solución.
for (i, j, t) in y:
    if y[i, j, t].x == 1:
        G_solution.add_edge(i, j)

# Dibujar el grafo.
#pos = nx.fruchterman_reingold_layout(G_solution)
pos = nx.spring_layout(G_solution)
nx.draw(G_solution, pos, with_labels=True, node_size=500, node_color='lightgray',  width=1, font_size=10)
plt.title("Grafo de la Solución Óptima", size=20)
plt.show()




