using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;

public class Solider : MonoBehaviour {

    private Vector3 scenePos;
    private Point maxCoord; 

    private void updatePos(Detection.PosAng newPosAng)
    {
        transform.position += newPosAng.pos;
        transform.localRotation = newPosAng.Ang;
    }

    // Use this for initialization
    void Start()
    {
        scenePos = transform.position;
    }



    // Update is called once per frame
    void LateUpdate()
    {
        if (Detection.currentPosAng.flag)
        {
            maxCoord = Detection.getMaxRenderingPoint(Detection.currentPosAng.pos.z * 15 - 15 + scenePos.z);
            transform.position = new Vector3(
                                    (Detection.currentPosAng.pos.x - 160) * (float)maxCoord.x / 160,
                                    (Detection.currentPosAng.pos.y - 120) * (float)maxCoord.y / 120,
                                    Detection.currentPosAng.pos.z * 15 - 15) + scenePos;

            transform.localRotation = Detection.currentPosAng.Ang;
            //Detection.currentPosAng.flag = false;
        }
    }
}
