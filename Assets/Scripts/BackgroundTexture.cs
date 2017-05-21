using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;

public class BackgroundTexture : MonoBehaviour {

    public GUITexture BTexture;
    private WebCamTexture CameraTexture;

    // Use this for initialization
    void Start()
    {
        BTexture = gameObject.AddComponent<GUITexture>();
        BTexture.pixelInset = new UnityEngine.Rect(0, 0, 0, 0);
        //set up camera
        WebCamDevice[] devices = WebCamTexture.devices;
        string backCamName = "";
        for (int i = 0; i < devices.Length; i++)
        {
            Debug.Log("Device:" + devices[i].name + "IS FRONT FACING:" + devices[i].isFrontFacing);

            if (!devices[i].isFrontFacing)
            {
                backCamName = devices[i].name;
            }
        }

        CameraTexture = new WebCamTexture(backCamName, 640, 320, 30);
        CameraTexture.Play();
        BTexture.texture = CameraTexture;

    }
    // Update is called once per frame
    void Update () {
		
	}
}
