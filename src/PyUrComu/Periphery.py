import numpy as np

def calculateHoles1D(P1,P2,dist,count):
    p1=P1[:2]
    p2=P2[:2]
    v_p1p2=p2-p1
    v_p1p2=v_p1p2/np.linalg.norm(v_p1p2)
    result=np.array([])
    for i in range(count):
        temp=p1+v_p1p2*dist*i
        temp=np.append(temp,[P1[2]])
        result=np.append(result,[temp])
    return result.reshape(count,3)

def calculateHoles2D(P1,P2,P3,dist,countx,county):
    p1=P1[:2]
    p2=P2[:2]
    p3=P3[:2]
    v_p1p2=p2-p1
    v_p1p2=v_p1p2/np.linalg.norm(v_p1p2)
    v_p1p3=p3-p1
    v_p1p3=v_p1p3/np.linalg.norm(v_p1p3)
    result=np.array([])
    for i2 in range(county):
        start=p1+v_p1p3*dist*i2
        for i in range(countx):
            temp=start+v_p1p2*dist*i
            result=np.append(result,[temp])

    return result.reshape(countx,2)