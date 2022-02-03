using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public Node nodeA;
    public Node nodeB;

    #endregion


    public float Length0;
    public float Length;

    private PhysicsManager Manager;

    // Update is called once per frame
    void Update () {

        Vector3 yaxis = new Vector3(0.0f, 1.0f, 0.0f);
        Vector3 dir = nodeA.Pos - nodeB.Pos;
        dir.Normalize();

        transform.position = 0.5f * (nodeA.Pos + nodeB.Pos);
        //The default length of a cylinder in Unity is 2.0
        transform.localScale = new Vector3(transform.localScale.x, Length / 2.0f, transform.localScale.z);
        transform.rotation = Quaternion.FromToRotation(yaxis, dir);
	}

    // Use this for initialization
    public void Initialize(PhysicsManager m)
    {
        Manager = m;

        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        Length = (nodeA.Pos - nodeB.Pos).magnitude;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // TO BE COMPLETED //

        //Carcular fuerza local del muelle


        force[nodeA.index] += -Stiffness * (Length - Length0)*((nodeB.Pos- nodeA.Pos).magnitude/Length);
       // force[nodeB.index] += -Stiffness * (Length - Length0) * ((nodeA.Pos - nodeB.Pos) / Length);
        

        //SUMAR esa fuerza a un nodo  y restarsela a otro
        //Hay que obtener los índices de los nodos
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx)
    {
        // TO BE COMPLETED //
        // matriz=vector.OuterProduct (Vector)
        //dFdx.SetSubMatrix(row,col,dFdx.SubMatrix(row, 3, col, 3 ) + dFadxa) /// Coger bloque y Sumar bloque 3x3
        MatrixXD dFadxa = new DenseMatrixXD(3);
        VectorXD vector = new DenseVectorXD(3);
        dFadxa = vector.OuterProduct(vector);
        
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);

        dFadxa = -Stiffness * ((Length - Length0) / Length) * I - Stiffness * (Length0 / Length);

        //dFdx.SetSubMatrix(row, col, dFdx.SubMatrix(row, 3, col, 3) + dFadxa);
        


    }

}
