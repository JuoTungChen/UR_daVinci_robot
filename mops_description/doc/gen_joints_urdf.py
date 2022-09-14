import numpy as np
import transformations as tf


transforms = []
ident = []

ident.append(('base', 'roll', 'revolute'))
transforms.append(
    np.array([[0,  0,  1,  0.4670],
              [0, -1,  0,  0],
              [1,  0,  0,  0],
              [0,  0,  0,  1]]))

ident.append(('roll', 'wrist', 'revolute'))
transforms.append(
    np.array([[0,  1,  0,  0],
              [0,  0,  1,  0],
              [1,  0,  0,  0],
              [0,  0,  0,  1]]))

ident.append(('wrist', 'jaw0', 'fixed'))
transforms.append(
    np.array([[1,  0,  0,  0.0091],
              [0,  0, -1,  0],
              [0,  1,  0,  0],
              [0,  0,  0,  1]]))

ident.append(('jaw0', 'jaw1', 'revolute'))
transforms.append(np.identity(4))

ident.append(('jaw0', 'jaw2', 'revolute'))
transforms.append(
    np.array([[1,  0,  0,  0],
              [0, -1,  0,  0],
              [0,  0, -1,  0],
              [0,  0,  0,  1]]))

ident.append(('jaw1', 'tcp1', 'fixed'))
transforms.append(
    np.array([[1,  0,  0,  0.009],
              [0,  1,  0,  0],
              [0,  0,  1,  0],
              [0,  0,  0,  1]]))

ident.append(('jaw2', 'tcp2', 'fixed'))
transforms.append(
    np.array([[1,  0,  0,  0.009],
              [0,  1,  0,  0],
              [0,  0,  1,  0],
              [0,  0,  0,  1]]))


for t, (parent, child, typ) in zip(transforms, ident):
    xyz = ' '.join(['{}'.format(n) for n in tf.translation_from_matrix(t)])
    rpy = ' '.join(['{}'.format(n) for n in tf.euler_from_matrix(t, 'sxyz')])

    print('<joint name="${{prefix}}{}_joint" type="{}">'.format(child, typ))
    print('  <parent link="${{prefix}}{}_link" />'.format(parent))
    print('  <child link="${{prefix}}{}_link" />'.format(child))
    print('  <origin xyz="{}" rpy="{}" />'.format(xyz, rpy))
    print('  <axis xyz="0 0 1" />')
    print('</joint>')
    print('')
