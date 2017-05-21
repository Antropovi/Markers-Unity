using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AllEnemies : MonoBehaviour {

    public Cyclop anotherOne;

	// Use this for initialization
	void Start () {
   //     StartCoroutine(spawning());
    }
	
	// Update is called once per frame
	void Update () {
        if (Input.GetButtonDown("Fire2"))
        {
            Cyclop clone = (Cyclop)Instantiate(anotherOne, new Vector3(0f,0f,25f), Quaternion.identity);
            clone.health = Random.Range(15, 75);
        }

    }

    private IEnumerator spawning() {

        yield return new WaitForSeconds(Random.Range(0.5f, 1f));
        while (Player.isAlive) {
            Cyclop clone = (Cyclop)Instantiate(anotherOne, new Vector3(Random.Range(-5f, 5f), Random.Range(-5f, 5f), 0f), Quaternion.identity);
            clone.health = Random.Range(15, 75);
            yield return new WaitForSeconds(Random.Range(0.5f, 1f));
        }
        yield return null;
    }
}
