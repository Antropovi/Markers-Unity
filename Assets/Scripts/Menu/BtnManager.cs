using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System;

public class BtnManager : MonoBehaviour {

    public void testBtn() {
        SceneManager.LoadScene("main", LoadSceneMode.Single);
    }
    public void calibrationMuneBtn()
    {
        SceneManager.LoadScene("calibrationMenu", LoadSceneMode.Single);
    }
    public void startCalibrationBtn() {
        foreach (Selectable selectableUI in Selectable.allSelectables)
        {
            if (selectableUI.tag == "field") {
                PlayerPrefs.SetInt(selectableUI.name,  Convert.ToInt32(((InputField)selectableUI).text));
            }
            if (selectableUI.tag == "dropdown") {
                PlayerPrefs.SetInt(selectableUI.name, ((Dropdown)selectableUI).value + 1);
            }            
        }
        SceneManager.LoadScene("calibrationProcess");
    }
    public void startBtn() {
        Calibration.mode = Calibration.Mode.CAPTURING;
    }
    public void tryAgainBtn()
    {
        SceneManager.LoadScene("main", LoadSceneMode.Single);
    }
    public void exit() {
        SceneManager.LoadScene("menu", LoadSceneMode.Single);
    }
}
