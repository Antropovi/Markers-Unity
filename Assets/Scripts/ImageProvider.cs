using UnityEngine;
using System.Collections;
using OpenCVForUnity;
using System;
using System.Linq;

public class WebcamImageProvider
{
    private bool isInit = false;
    private WebCamTexture texture = null;

    public WebCamTexture webcamTexture
    {
        get { return texture; }
    }
    Color32[] bufferPixels = null;

    public static bool IsAvailable
    {
        get
        {
            return WebCamTexture.devices.Length > 0;
        }
    }

    public void Init()
    {
        if (!isInit)
        {
            WebCamDevice[] devices = WebCamTexture.devices;
            string backCamName = "";
            for (int i = 0; i < devices.Length; i++)
            {
                if (!devices[i].isFrontFacing)
                {
                    backCamName = devices[i].name;
                }
            }

            texture = new WebCamTexture(backCamName, 640, 480, 40);
            texture.Play();
            isInit = true;
        }
    }

    public IEnumerator GetImage(Mat pixels)
    {
        if (!isInit) Init();

        while (!texture.didUpdateThisFrame) yield return new WaitForEndOfFrame();

        if (bufferPixels == null)
            bufferPixels = new Color32[texture.width * texture.height];

        pixels.create(texture.height, texture.width, CvType.CV_8UC4);
        Utils.webCamTextureToMat(texture, pixels, bufferPixels);

        yield break;
    }
}
