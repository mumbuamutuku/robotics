entry_points={
    'console_scripts': [
        'yolo_finetune = adrr_development.yolo_finetune:main',
        'navigation = adrr_development.ddpg_navigation:main',
        'ethical_decision = adrr_development.ethical_mdp:main',
        'energy_scheduler = adrr_development.energy_scheduler:main',
        'testing = adrr_development.ros_testing:main',
    ],
},