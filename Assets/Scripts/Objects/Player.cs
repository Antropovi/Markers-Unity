using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class Player : MonoBehaviour {

    public static bool isAlive;
    private static int hp;
    public static int PlayerHealth
    {
        set {
            GameObject.Find("Health").GetComponent<Text>().text = value.ToString();
            hp = value;
        }
        get { return hp; }
    }

    private static float score;
    public static float Score {
        set { GameObject.Find("Score").GetComponent<Text>().text = value.ToString();
            score = value;
        }
        get { return score; }
    }

    // Use this for initialization
    void Start () {
        Score = 0;
        PlayerHealth = 100;
        isAlive = true;
	}

    // Update is called once per frame
    void Update()
    {
        if (hp < 0)
        {
            isAlive = false;
            PlayerPrefs.SetFloat("Score", score);
            SceneManager.LoadScene("Score");
        }
    }
}
