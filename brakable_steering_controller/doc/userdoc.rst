:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/bicycle_steering_controller/doc/userdoc.rst

.. _brakable_steering_controller_userdoc:

brakable_steering_controller
=============================

This controller extends functionality of bicycle_steering_controller by adding brake to decelerate faster.

Parameters
,,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

For an exemplary parameterization see the ``test`` folder of the controller's package.

Additionally to the parameters of the :ref:`Steering Controller Library <doc/ros2_controllers/steering_controllers_library/doc/userdoc:parameters>`, this controller adds the following parameters:

.. generate_parameter_library_details:: ../src/brakable_steering_controller.yaml
