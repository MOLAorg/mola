.. _tutorial-mulran-replay-to-ros2:

=======================================
Replay MulRan to ROS2
=======================================

This tutorial will guide you through the process of using MOLA to publish a sequence of the MulRan dataset (:cite:`gskim-2020-mulran`)
as ROS 2 topics, so they can be visualized in RViz, FoxGlove, or used in your own system.

.. note::

  Users can also access to all MulRan dataset observations directly with the C++ API of :ref:`mola::MulranDataset <doxid-classmola_1_1_mulran_dataset>`.


.. raw:: html

   <div style="width: 100%; overflow: hidden;">
     <video controls autoplay loop muted style="width: 512px;">
       <source src="https://mrpt.github.io/videos/mola_demos_Mulran_replay_to_ros2.mp4" type="video/mp4">
     </video>
   </div>

|

MOLA installation
----------------------------------
This tutorial requires the installation of these MOLA packages: ``mola_demos``, ``mola_input_mulran_dataset``, ``mola_viz``.

Following the default :ref:`installation instructions <installing>` is enough.

|

Dataset preparation
----------------------------------
Download the MulRan dataset (:cite:`gskim-2020-mulran`) from their `website <https://sites.google.com/view/mulran-pr/>`_,
and extract the files anywhere in your system such as the files layout is as expected by :ref:`mola::MulranDataset <doxid-classmola_1_1_mulran_dataset>`.

Remember setting the environment variable ``MULRAN_BASE_DIR`` to the root directory of your dataset, for example: 

.. code-block:: bash

    export MULRAN_BASE_DIR=/home/myuser/mulran/

|

Launching the demo
----------------------------------

To launch the demo, just run:

.. code-block:: bash

    # Run the default KAIST01 sequence:
    ros2 launch mola_demos ros-mulran-play.launch.py

    # Or select another sequence:
    ros2 launch mola_demos ros-mulran-play.launch.py  mulran_sequence:=DCC02

|

Under the hook
----------------------------------
This demo comprises two main files:

1) The **MOLA system configuration** file. It defines two MOLA modules: the MulRan input and the ROS2 bridge.

.. dropdown:: YAML listing
    :icon: code-review

    File: `mulran_just_replay_to_ros2.yaml <https://github.com/MOLAorg/mola/blob/develop/mola_demos/mola-cli-launchs/mulran_just_replay_to_ros2.yaml>`_

    .. literalinclude:: ../../mola_demos/mola-cli-launchs/mulran_just_replay_to_ros2.yaml
       :language: yaml

2) The **ROS 2 launch** file. It invokes ``mola-cli`` and ``rviz2``.

.. dropdown:: Launch file listing
    :icon: code-review

    File: `ros-mulran-play.launch.py <https://github.com/MOLAorg/mola/blob/develop/mola_demos/ros2-launchs/ros-mulran-play.launch.py>`_

    .. literalinclude:: ../../mola_demos/ros2-launchs/ros-mulran-play.launch.py
       :language: python
