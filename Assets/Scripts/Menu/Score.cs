using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Score : MonoBehaviour {

	// Use this for initialization
	void Start () {
        this.GetComponent<Text>().text = "Final score: " + PlayerPrefs.GetFloat("Score").ToString();
	}
}
