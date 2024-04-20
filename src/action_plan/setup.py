maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',

entry_points={
        'console_scripts': [
                'talker = robot_action.robot_action:main',
                'listener = robot_action.subscriber_member_function:main',
        ],
},
