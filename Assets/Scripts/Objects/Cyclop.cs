using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity;

public class Cyclop : MonoBehaviour
{

    private float runSpeed = 3f;
    private float startTime;
    private float journeyLength;
    private Vector3 newCoordSys;
    private Vector3 finish;
    private float rotationSpeed = 0.7f;

    private Vector3 scenePos;
    private Point maxCoord;
    public int health;
    private Animation animation;
    private bool isDead;
    private int fullHp;
    float fracJourney;
    private float lastAttackTime;

    // Use this for initialization
    void Start()
    {
        animation = this.GetComponentInChildren<Animation>();
        animation["death"].wrapMode = WrapMode.Once;
        animation["walk"].wrapMode = WrapMode.Once;
        animation["idle"].wrapMode = WrapMode.Once;
        scenePos = transform.position;
        isDead = false;
        fullHp = health;
        //start = transform.position;
        startTime = Time.time;
        finish = new Vector3(Random.Range(-4f, 4f), -1.5f, -7f);
        journeyLength = Vector3.Distance(scenePos, finish);
        // StartCoroutine(run());    
    }

    void getHitted()
    {
        animation.Stop();
        animation.Play("hit");
        health -= Random.Range(10, 35);
    }

    // Update is called once per frame
    void Update()
    {
        maxCoord = Detection.getMaxRenderingPoint(Detection.currentPosAng.pos.z * 15 - 15 + transform.position.z);
        //Debug.Log(maxCoord.x + "  |  " + maxCoord.y);
        if (Detection.currentPosAng.flag)
        {       
            newCoordSys = new Vector3(
                                    (Detection.currentPosAng.pos.x - 160) * (float)maxCoord.x / 160,
                                    (Detection.currentPosAng.pos.y - 120) * (float)maxCoord.y / 120,
                                    Detection.currentPosAng.pos.z * 15 - 15);
            
            //transform.position = scenePos;

            transform.localRotation = Detection.currentPosAng.Ang;
        }


        var rotationAngle = Quaternion.LookRotation(transform.position - new Vector3(0f, 0f, -15f));
        transform.localRotation = Quaternion.Slerp(transform.rotation, rotationAngle, Time.deltaTime * rotationSpeed);



        if (!isDead)
        {
            if (fracJourney < 1)
            {
                float distCovered = (Time.time - startTime) * runSpeed;
                fracJourney = distCovered / journeyLength;
                journeyLength = Vector3.Distance(scenePos + newCoordSys, finish);
                transform.position = Vector3.Lerp(scenePos + newCoordSys, finish, fracJourney);

                animation.CrossFadeQueued("walk");
            }
            else
            {
                transform.position = new Vector3((Detection.currentPosAng.pos.x - 160) * (float)maxCoord.x / 320 + finish.x,
                                    (Detection.currentPosAng.pos.y - 120) * (float)maxCoord.y / 240 + finish.y, transform.position.z);

                animation.CrossFadeQueued("idle");
                if (Time.time - lastAttackTime > Random.Range(3f, 10f))
                {
                    hitPlayer();
                    lastAttackTime = Time.time;
                }
            }
        }

        if (!isDead && health <= 0)
        {
            animation.Stop();
            StartCoroutine(dead());
            isDead = true;
        }
    }

    private void hitPlayer() {
            animation.Stop();
            animation.Play("attack_" + Random.Range(1, 3));
            Player.PlayerHealth -= Random.Range(5, 10);
    }


    private IEnumerator dead()
    {
        Player.Score += fullHp / 10;
        animation.Play("death");
        this.GetComponentInChildren<MeshCollider>().enabled = false;
        yield return new WaitUntil(() => !animation.isPlaying);

        DestroyObject(this.gameObject);
        isDead = true;
        yield return null;
    }


    bool playAnim(string name)
    {
        animation.Play("death");
        this.GetComponentInChildren<MeshCollider>().enabled = false;
        return true;
    }
}
