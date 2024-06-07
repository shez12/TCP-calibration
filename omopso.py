from pyMultiobjective.algorithm.omopso import optimized_multiobjective_particle_swarm_optimization
from sympy import symbols




def omopsoCompute(list_of_functions):
    swarm_size= 20
    min_values = (-5,9, -80)
    max_values =  (0, 10,-81)
    mutation_rate = 0.1       
    iterations =  100
    eta = 1
    verbose =  True


    return optimized_multiobjective_particle_swarm_optimization(swarm_size, min_values, max_values , iterations , list_of_functions, mutation_rate, eta, verbose)

    
