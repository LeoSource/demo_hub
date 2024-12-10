import torch
import matplotlib.pyplot as plt
from chapter13_4 import multibox_prior,show_bboxes


def display_anchors(fmap_w, fmap_h, s):
    # d2l.set_figsize()
    # 前两个维度上的值不影响输出
    fmap = torch.zeros((1, 10, fmap_h, fmap_w))
    anchors = multibox_prior(fmap, sizes=s, ratios=[1, 2, 0.5])
    bbox_scale = torch.tensor((w, h, w, h))
    # print(anchors)
    # print(anchors[0].shape)
    show_bboxes(plt.imshow(img).axes,
                    anchors[0] * bbox_scale)

if __name__ == '__main__':
    img = plt.imread('./img/catdog.jpg')
    h, w = img.shape[:2]
    print(h, w)

    # display_anchors(fmap_w=4, fmap_h=4, s=[0.15])
    # display_anchors(fmap_w=2, fmap_h=2, s=[0.4])
    display_anchors(fmap_w=1, fmap_h=1, s=[0.8])

    plt.show()