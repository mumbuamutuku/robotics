entry_points={
    'console_scripts': [
        'vit_finetune = warehouse_assistant.vit_finetune:main',
        'cartographer_slam = warehouse_assistant.cartographer_slam:main',
        'evaluation_kpis = warehouse_assistant.evaluation_kpis:main',
        'safety_module = warehouse_assistant.safety_module:main',
        'rl_grasp = warehouse_assistant.rl_grasp:main',
        'hierarchical_planner = warehouse_assistant.hierarchical_planner:main',
        'ekf_fusion = warehouse_assistant.ekf_fusion:main',
    ],
},