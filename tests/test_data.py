import csv
from argparse import Namespace
import pytest
import cv2
import numpy as np
from objectpilot.data import prepare

def manifest(tmp_path, changes=None):
    rows=[]
    for i,split in enumerate(('train','val','test')):
        image=tmp_path/f'{i}.jpg'; label=tmp_path/f'{i}.txt'
        cv2.imwrite(str(image),np.full((40,40,3),i*60,np.uint8))
        label.write_text('0 0.5 0.5 0.3 0.3\n')
        rows.append(dict(image=str(image),label_path=str(label),label_reviewed='yes',group_id=str(i),split=split))
    if changes: changes(rows)
    path=tmp_path/'frames.csv'
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]); w.writeheader(); w.writerows(rows)
    return Namespace(frames=str(path),out=str(tmp_path/'dataset'),classes='bottle')

def test_prepare_real_labels_and_groups(tmp_path):
    args=manifest(tmp_path); prepare(args)
    assert (tmp_path/'dataset/dataset.yaml').is_file()
    assert len(list((tmp_path/'dataset/labels/train').glob('*.txt')))==1

def test_reject_cross_split_group(tmp_path):
    args=manifest(tmp_path,lambda rows:rows[1].update(group_id='0'))
    with pytest.raises(ValueError,match='Group leakage'): prepare(args)

def test_reject_unreviewed_label(tmp_path):
    args=manifest(tmp_path,lambda rows:rows[0].update(label_reviewed='no'))
    with pytest.raises(ValueError,match='Unreviewed'): prepare(args)

def test_reject_duplicate_images(tmp_path):
    args=manifest(tmp_path,lambda rows:rows[1].update(image=rows[0]['image']))
    with pytest.raises(ValueError,match='Duplicate image'): prepare(args)
