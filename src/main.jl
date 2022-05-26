include("environment.jl")
include("utils.jl")
include("hybrid_a_star.jl")


function generate_cart_path(world)

    sequence_of_control_inputs = hybrid_a_star_search(world.cart.x,world.cart.y,world.cart.theta,world.cart.goal.x,world.cart.goal.y,world)
    world.cart_hybrid_astar_path = sequence_of_control_inputs
    starting_state = [world.cart.x,world.cart.y,world.cart.theta]
    path_x,path_y,path_theta = [world.cart.x],[world.cart.y],[world.cart.theta]

    for control_input in sequence_of_control_inputs
        extra_parameters = [1.0, 1.0, control_input]
        x,y,theta = get_intermediate_points(starting_state, 1.0, extra_parameters);
        push!(path_x,last(x))
        push!(path_y,last(y))
        push!(path_theta,last(theta))
        starting_state = [last(x),last(y),last(theta)]
    end

    return path_x,path_y,path_theta
end


function generate_gif(world,path_x,path_y,path_theta)

    all_environments = Array{experiment_environment,1}()
    for i in 1:length(path_x)
        temp_world = deepcopy(world)
        temp_world.cart.x = path_x[i]
        temp_world.cart.y = path_y[i]
        temp_world.cart.theta = path_theta[i]
        temp_world.cart_hybrid_astar_path = world.cart_hybrid_astar_path[i:end]
        push!(all_environments,temp_world)
    end

    anim = @animate for i in 1:length(all_environments)
        display_env(all_environments[i],i);
    end
    gif(anim, "path.gif", fps = 10)
end
