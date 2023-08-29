import rosbag
import rospy
from tf_bag import BagTfTransformer
import numpy as np
import matplotlib.pyplot as plt

bag_names = ['demo1', 'demo2', 'demo3', 'demo5']
ax = plt.figure().add_subplot(projection='3d')

for name in bag_names:
    print(name)
    bag_file_path = name + '.bag'
    name = name + 'base'

    bag = rosbag.Bag(bag_file_path)

    bag_transformer = BagTfTransformer(bag)

    #print(bag_transformer.getTransformGraphInfo())

    ft_time = bag_transformer.waitForTransform('base_link', 'link_grasp_center')

    print(ft_time)
    translation, quaternion = bag_transformer.lookupTransform('base_link', 'link_grasp_center', ft_time)
    trans = [translation]

    for i in range(10, 1000):
        time = bag_transformer.getTimeAtPercent(i/10.)
    
        translation, quaternion = bag_transformer.lookupTransform('base_link', 'link_grasp_center', time)
        #print(time, translation, quaternion)
        trans.append(translation)
    
    traj = np.array(trans)
    print(np.shape(traj))
    np.savetxt(name + '_xyz.txt', traj)



    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label=name)
ax.legend()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
