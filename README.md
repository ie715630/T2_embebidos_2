Nuestro algoritmo, de acuerdo al algoritmo de Rate Monotonic,
empieza a buscar si alguna tarea tiene la mayor prioridad (PRIORITY_0).
Si ninguna tarea la tiene este empieza a buscar para la siguiente prioridad (PRIORITY_1),
hasta encontrar la siguiente tarea a ejecutar.

Una condicion para que se debe cumplir para que la tarea se puede ejecutar es que esta no este
en estado waiting. Asi mismo si ninguna tarea esta disponible por defecto la tarea que siempre
estara disponible y no tiene prioridad es la task Idle. 
