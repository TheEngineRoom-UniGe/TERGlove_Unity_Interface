using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// public GameObject DIP;

public class DIP_aproximation : MonoBehaviour
{
    public GameObject PIP;
    public float amount = 0.7f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Quaternion q = PIP.transform.localRotation;
        this.transform.localRotation = Quaternion.Lerp(Quaternion.identity, q, amount);
    }
}
