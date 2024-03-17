using System.Collections;
using System.Collections.Generic;
using UnityEngine;




public class QuadTree
{
    public enum ORDER
    {
        LEFT_TOP,
        LEFT_BOTTOM,
        RIGHT_TOP,
        RIGHT_BOTTOM
    }

    public const int MAX_DEPTH = 10;

    public int depth;
    public float state = 0;
    public Rect rect;
    public QuadTree parent;
    public QuadTree[] children = new QuadTree[4];
    public GameObject box = null;
    Vector2 sensorPos = new Vector2(0, -2500);

    public QuadTree()
    {

    }

    public QuadTree(int depth,Rect rect)
    {
        this.depth = depth;
        this.rect = rect;
        //Debug.Log(this.rect.center);
        //Debug.Log(this.rect.yMin);
    }

    public QuadTree(QuadTree parent,int order)
    {
        this.parent = parent;
        this.depth = parent.depth + 1;

        float x = 0, y = 0;

        if (order == (int)ORDER.LEFT_TOP)
        {
            x = parent.rect.xMin;
            y = parent.rect.center.y;            
        }
        else if (order == (int)ORDER.LEFT_BOTTOM)
        {
            x = parent.rect.xMin;
            y = parent.rect.yMin;
        }
        else if (order == (int)ORDER.RIGHT_TOP)
        {
            x = parent.rect.center.x;
            y = parent.rect.center.y;
        }
        else if (order == (int)ORDER.RIGHT_BOTTOM)
        {
            x = parent.rect.center.x;
            y = parent.rect.yMin;
        }

        this.rect = new Rect(x, y, parent.rect.width / 2.0f, parent.rect.height / 2.0f);
    }


    bool CheckVectorInRect(Vector2 pos,Rect rect)
    {
        if (pos.x > rect.xMin && pos.x < rect.xMax && pos.y > rect.yMin && pos.y < rect.yMax) return true;
        return false;
    }



    public void checkRay(float angle, float distance, MainScript mainScript)
    {

        //자기 자신이 ray와 교차하는지 체크한다. 
        Vector2 dir = new Vector2(distance * Mathf.Cos(angle * Mathf.PI / 180.0f),distance * Mathf.Sin(angle * Mathf.PI / 180.0f));
        Vector2 target = sensorPos + dir;

        bool isInRect = CheckVectorInRect(target, this.rect);        

        if (!isInRect)
        {
            //4개의 선분을 체크한다. 
            if (!FasterLineSegmentIntersection(sensorPos, target, new Vector2(rect.xMin, rect.yMin), new Vector2(rect.xMax, rect.yMin)))
            {
                if (!FasterLineSegmentIntersection(sensorPos, target, new Vector2(rect.xMin, rect.yMin), new Vector2(rect.xMin, rect.yMax)))
                {
                    if (!FasterLineSegmentIntersection(sensorPos, target, new Vector2(rect.xMin, rect.yMax), new Vector2(rect.xMax, rect.yMax)))
                    {
                        if (!FasterLineSegmentIntersection(sensorPos, target, new Vector2(rect.xMax, rect.yMin), new Vector2(rect.xMax, rect.yMax)))
                        {
                            if (box != null)
                            {
                                box.SetActive(false);
                            }
                            return;
                        }
                    }
                }
            }
        }

        if (depth == MAX_DEPTH)
        {
            if (isInRect)
            {
                state += 0.4f;
                if (state > 1) state = 1;

                if (state > 0)
                {
                    if (box == null)
                    {
                        box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        box.transform.position = new Vector3(rect.center.x,0,rect.center.y);
                        box.transform.localScale = new Vector3(rect.width, 1, rect.height);                        
                    }

                    box.GetComponent<MeshRenderer>().material = mainScript.red;
                    box.SetActive(true);                    
                }
            }
            else
            {                                
                state -= 0.4f;
                if (state < -1) state = -1;

                if (box == null)
                {
                    box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    box.transform.position = new Vector3(rect.center.x, 0, rect.center.y);
                    box.transform.localScale = new Vector3(rect.width, 1, rect.height);
                }

                box.GetComponent<MeshRenderer>().material = mainScript.blue;
                box.SetActive(true);
            }

            return;
        }


        for (int i = 0; i < children.Length; i++)
        {
            if (children[i] == null)
            {
                children[i] = new QuadTree(this, i);
            }

            children[i].checkRay(angle, distance,mainScript);
        }

    }

    bool FasterLineSegmentIntersection(Vector2 line1point1, Vector2 line1point2, Vector2 line2point1, Vector2 line2point2)
    {
        Vector2 a = line1point2 - line1point1;
        Vector2 b = line2point1 - line2point2;
        Vector2 c = line1point1 - line2point1;

        float alphaNumerator = b.y * c.x - b.x * c.y;
        float betaNumerator = a.x * c.y - a.y * c.x;
        float denominator = a.y * b.x - a.x * b.y;

        if (denominator == 0)
        {
            return false;
        }
        else if (denominator > 0)
        {
            if (alphaNumerator < 0 || alphaNumerator > denominator || betaNumerator < 0 || betaNumerator > denominator)
            {
                return false;
            }
        }
        else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
        {
            return false;
        }
        return true;
    }

}

public class MainScript : MonoBehaviour
{
    QuadTree root = new QuadTree(0, new Rect(-2500,-2500,5000,5000));
    Vector2 sensorPos = new Vector2(0, -2500);

    public Material blue;
    public Material red;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnMessageArrived(string msg)
    {
        Debug.Log(msg);

        string[] datas = msg.Split(",");

        float distance = 0;
        float angle = 0;
        if (!float.TryParse(datas[0], out distance)) return;
        if (!float.TryParse(datas[1], out angle)) return;        

        root.checkRay(angle, distance,this);
    }


    void OnConnectionEVent(bool success)
    {
        Debug.Log("connected:" + success);


    }
}
