entry_points={
    'console_scripts': [
        'slam_node = drone_delivery.slam_node:main',
        'obstacle_detection = drone_delivery.obstacle_detection:main',
        'drone_architecture = drone_delivery.drone_architecture:main',
        'mpc_controller = drone_delivery.mpc_controller:main',
        'rl_path_planning = drone_delivery.rl_path_planning:main',
        'marl_swarm = drone_delivery.marl_swarm:main',
        'vae_fault_detection = drone_delivery.vae_fault_detection:main',
    ],
},