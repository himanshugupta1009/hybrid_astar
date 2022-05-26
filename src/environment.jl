using Plots
using Random

#Global Variables
plot_size = 1000; #number of pixels
cart_size = 1; # radius in meters

#Various different Struct definitions
struct location
    x::Float64
    y::Float64
end

mutable struct human_state
    x::Float64
    y::Float64
    v::Float64
    goal::location
    id::Float64
end

struct obstacle_location
    x::Float64
    y::Float64
    r::Float64 #Radius of the obstacle which is assumed to be a circle
end

mutable struct cart_state
    x::Float64
    y::Float64
    theta::Float64
    v::Float64
    L::Float64
    goal::location
end

struct human_probability_over_goals
    distribution::Array{Float64,1}
end

mutable struct experiment_environment
    length::Float64
    breadth::Float64
    obstacles::Array{obstacle_location,1}
    cart::cart_state
    cart_start_location::location
    cart_hybrid_astar_path::Array{Float64,1}
end

#Define the Environment
function generate_environment(world_length, world_breadth, cart_start_position, cart_goal_position, obstacle_list)

    all_obstacle_list = Array{obstacle_location,1}()
    for o in obstacle_list
        obstacle = obstacle_location(o[1],o[2],o[3])
        push!(all_obstacle_list,obstacle)
    end

    cart_goal = location(cart_goal_position[1],cart_goal_position[2])
    cart = cart_state(cart_start_position[1],cart_start_position[2],cart_start_position[3],0.0,1.0,cart_goal)
    world = experiment_environment(world_length,world_breadth,all_obstacle_list,cart,location(cart_start_position[1], cart_start_position[2]),Float64[])
    return world
end

#Function to display the environment
function display_env(env::experiment_environment, time_stamp=nothing)

    #Plot Boundaries
    # p = plot([0.0],[0.0],legend=false,grid=false)
    p = plot([0.0],[0.0],legend=false,grid=false,axis=([], false))
    plot!([env.length], [env.breadth],legend=false)

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot Golfcart
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="blue", msize= 0.3*plot_size*cart_size/env.length)
    quiver!([env.cart.x],[env.cart.y],quiver=([cos(env.cart.theta)],[sin(env.cart.theta)]), color="blue")

    #Plot the Hybrid A* path if it exists
    if(length(env.cart_hybrid_astar_path)!=0)
        initial_state = [env.cart.x,env.cart.y,env.cart.theta]
        path_x, path_y = [env.cart.x],[env.cart.y]
        for steering_angle in env.cart_hybrid_astar_path
            extra_parameters = [1.0, env.cart.L, steering_angle]
            x,y,theta = get_intermediate_points(initial_state, 1.0, extra_parameters);
            for pos_x in 2:length(x)
                push!(path_x,x[pos_x])
            end
            for pos_y in 2:length(y)
                push!(path_y,y[pos_y])
            end
            initial_state = [last(x),last(y),last(theta)]
        end
        plot!(path_x,path_y,color="brown")
    end

    if(time_stamp!=nothing)
        annotate!(env.length/2, env.breadth, text("t="*string(time_stamp), :blue, :right, 15))
    end
    annotate!(env.cart_start_location.x, env.cart_start_location.y, text("START", :green, :right, 10))
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :green, :right, 10))
    plot!(size=(plot_size,plot_size))
    display(p)
end
