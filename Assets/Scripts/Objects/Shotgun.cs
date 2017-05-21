using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Shotgun : MonoBehaviour
{


    Animator shotgunAnim;
    // Use this for initialization
    void Start()
    {
        shotgunAnim = GetComponent<Animator>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetButtonDown("Fire1"))
        {
            if (shotgunAnim.GetCurrentAnimatorStateInfo(0).fullPathHash != Animator.StringToHash("Base Layer.Shotgun_Reload"))
            {
                shotgunAnim.SetTrigger("Reload");
                RaycastHit hit = new RaycastHit();
                if (Physics.Raycast(Camera.main.transform.position, Camera.main.transform.TransformDirection(Vector3.forward), out hit))
                {
                    if (hit.distance < 25f)
                    {
                        hit.transform.SendMessageUpwards("getHitted", SendMessageOptions.DontRequireReceiver);
                    }
                }
            }
        }
    }
}
