#Load Required Packages
using DifferentialEquations
using DataStructures
using Random
using LinearAlgebra
include("utils.jl")

mutable struct graph_node
    x::Float64
    y::Float64
    theta::Float64
    actual_cost::Float64
    heuristic_cost::Float64
    action_taken_to_reach_here::Float64
    discrete_x::Float64
    discrete_y::Float64
    discrete_theta::Float64
    parent::Union{graph_node,Nothing}
    time_stamp::Float64
end

function is_goal(node, goal_x, goal_y, threshold)
    euclidean_distance =  ( (node.x - goal_x)^2 + (node.y - goal_y)^2 )^ 0.5
    if(euclidean_distance < threshold)
        return true
    else
        return false
    end
end

function calculate_heuristic_cost(node_x, node_y, node_theta, goal_x, goal_y, world)
    euclidean_distance =  ( (node_x - goal_x)^2 + (node_y - goal_y)^2 )^ 0.5
    direct_line_to_goal_slope = wrap_between_0_and_2Pi(atan(goal_y-node_y,goal_x-node_x))
    orientation_cost = 10* dot( (cos(direct_line_to_goal_slope), sin(direct_line_to_goal_slope)) ,
                                (cos(node_theta), sin(node_theta)) )
    return euclidean_distance - orientation_cost
end

function get_path(current_node)
    steering_angle_controls_sequence = []
    @show(current_node.actual_cost)
    while(current_node.parent!= nothing)
        push!(steering_angle_controls_sequence, current_node.action_taken_to_reach_here)
        current_node = current_node.parent
    end
    return reverse(steering_angle_controls_sequence)
end

function get_new_x_y_theta(current_x, current_y, current_theta, steering_angle,time_interval, env, arc_length)
    if(steering_angle == 0.0)
        new_theta = current_theta
        new_x = current_x + arc_length*cos(current_theta)*time_interval
        new_y = current_y + arc_length*sin(current_theta)*time_interval
    else
        new_theta = current_theta + (arc_length * tan(steering_angle) * time_interval / env.cart.L)
        new_theta = wrap_between_0_and_2Pi(new_theta)

        new_x = current_x + ((env.cart.L / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
        new_y = current_y + ((env.cart.L / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
    end
    return float(new_x), float(new_y), float(new_theta)
end

function get_discrete_state(environment, x, y)
    discretization_width = 0.5
    max_num_bins_x = ceil(environment.length/discretization_width)
    discrete_x = clamp(ceil(x/discretization_width),1,max_num_bins_x)
    max_num_bins_y = ceil(environment.breadth/discretization_width)
    discrete_y = clamp(ceil(y/discretization_width),1,max_num_bins_y)
    return discrete_x::Float64,discrete_y::Float64
end

function get_action_cost(environment, final_x::Float64, final_y::Float64, obs_cost::Float64, action::Float64)
    total_cost::Float64 = 0.0

    #Cost from going out of bounds
    if(final_x<=0.0 || final_x>=environment.length)
        return Inf
    end
    if(final_y<=0.0 || final_y>=environment.breadth)
        return Inf
    end

    #Cost from obstacles
    padding_radius = 2.0 + environment.cart.L
    for obstacle in environment.obstacles
        euclidean_distance::Float64 = ( (final_x - obstacle.x)^2 + (final_y - obstacle.y)^2 )^ 0.5
        if(euclidean_distance >= obstacle.r + padding_radius)
            continue
        else
            return Inf
        end
    end

    #Cost from no change in steering angle
    if(action == 0.0)
       total_cost += -1.0
    end

    #Cost from Long Paths
    total_cost += 1

    return total_cost
end

function hybrid_a_star_search(start_x, start_y, start_theta, goal_x, goal_y, env, time_limit=0.2)

    #Action Set
    set_of_delta_angles = Array{Float64,1}([0.0])
    for i in 1:9
        push!(set_of_delta_angles, float(-5*i*pi/180))
        push!(set_of_delta_angles, float(5*i*pi/180))
    end

    #delta_t = 1 second
    time_step = 1
    #Radius of circle around goal = 1m
    radius_around_goal_threshold = 1.0
    #Obstacle collision threshold
    obstacle_collision_cost = 1.0
    #Distance covered in one time interval
    arc_length = 1.0
    #Factor to give less weight to distant points
    lambda = 1.0

    path = Array{Float64,1}()
    open = PriorityQueue{String,Float64}(Base.Order.Forward)
    closed = Dict{String,graph_node}()
    dict_of_nodes = Dict{String,graph_node}()
    start_theta = wrap_between_0_and_2Pi(start_theta)
    start_discrete_x, start_discrete_y  = get_discrete_state(env,start_x,start_y)
    start_discrete_theta = ceil(start_theta*180/pi)

    start_node = graph_node(start_x, start_y, start_theta, 0.0,
        calculate_heuristic_cost(start_x,start_y,start_theta,goal_x,goal_y,env), -100.0,
        start_discrete_x, start_discrete_y, start_discrete_theta, nothing, 0.0)

    node_key = "x"*string(start_node.discrete_x)*"y"*string(start_node.discrete_y)*"theta"*string(start_discrete_theta)
    open[node_key] = (lambda^start_node.time_stamp)*start_node.actual_cost + start_node.heuristic_cost
    dict_of_nodes[node_key] = start_node
    start_time = time()

    while(length(open) != 0)
        current_node = dict_of_nodes[dequeue!(open)]
        if(is_goal(current_node, goal_x, goal_y, radius_around_goal_threshold))
            path = get_path(current_node)
            return path
        end

        node_key = "x"*string(current_node.discrete_x)*"y"*string(current_node.discrete_y)*"theta"*string(current_node.discrete_theta)
        closed[node_key] = current_node
        current_state = [current_node.x,current_node.y,current_node.theta]

        for delta in set_of_delta_angles
            steering_angle = atan((env.cart.L*delta)/arc_length)
            final_x,final_y,final_theta = get_new_x_y_theta(current_node.x, current_node.y,
                current_node.theta, steering_angle, time_step, env, arc_length)
            discrete_x, discrete_y = get_discrete_state(env,final_x,final_y)
            discrete_theta = current_node.discrete_theta + round(delta*180/pi)
            node_key = "x"*string(discrete_x)*"y"*string(discrete_y)*"theta"*string(discrete_theta)
            if(haskey(closed,node_key))
                continue
            end
            g = current_node.actual_cost + (lambda^(current_node.time_stamp+1))* get_action_cost(env,final_x, final_y, obstacle_collision_cost,steering_angle)
            h = calculate_heuristic_cost(final_x, final_y, final_theta, goal_x, goal_y, env)
            new_node = graph_node(final_x, final_y, final_theta, g, h,
                steering_angle, discrete_x, discrete_y,discrete_theta, current_node,current_node.time_stamp+1)
            #println(new_node)
            if(new_node.actual_cost == Inf)
                closed[node_key] = new_node
                continue
            end
            if(haskey(open,node_key))
                if(dict_of_nodes[node_key].actual_cost > new_node.actual_cost)
                    dict_of_nodes[node_key] = new_node
                    open[node_key] = new_node.heuristic_cost + new_node.actual_cost
                end
            else
                dict_of_nodes[node_key] = new_node
                open[node_key] = new_node.heuristic_cost + new_node.actual_cost
            end
        end
        if(time()- start_time >= time_limit)
            @show("Time exceeded")
            return path
        end
    end
    return path
end
