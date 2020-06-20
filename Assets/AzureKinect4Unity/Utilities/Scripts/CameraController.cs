//
// This is a modified version of the source code from Es's blog.
//
// The original source code is available in Es's blog.
// http://esprog.hatenablog.com/entry/2016/03/20/033322
//

using UnityEngine;

public class CameraController : MonoBehaviour
{
	[SerializeField] float _TrackSpeed = 2.0f;
	[SerializeField] float _MoveSpeed = 0.5f;
	[SerializeField] float _RotateSpeed = 2.0f;

	public enum MouseButtonType
	{
		Left,
		Middle,
		Right,
		None,
	}

	void Update()
	{
#if UNITY_EDITOR || UNITY_STANDALONE
		Track();
		MouseButtonType buttonType = GetInputMouseButton();
		switch(buttonType)
		{
			case MouseButtonType.Left:
				break;
			case MouseButtonType.Right:
				Rotate();
				break;
			case MouseButtonType.Middle:
				Move();
				break;
			default:
				break;
		}
#endif
	}

	private void Track()
	{
		float scroll = Input.GetAxis("Mouse ScrollWheel");
		transform.position += transform.forward * _TrackSpeed * scroll; 
	}

	private void Rotate()
	{
		Vector2 angle = Vector2.zero;
		angle.x = Input.GetAxis("Mouse X");
		angle.y = Input.GetAxis("Mouse Y");
    	transform.RotateAround(transform.position, Vector3.up, _RotateSpeed * angle.x);
		transform.RotateAround(transform.position, transform.right, -_RotateSpeed * angle.y);
	}

	private void Move()
	{
		Vector3 horizontal = transform.right * (-Input.GetAxis("Mouse X")) * _MoveSpeed;
		Vector3 vertical = transform.up * (-Input.GetAxis("Mouse Y")) * _MoveSpeed;
		transform.position += (horizontal + vertical);
	}

	private MouseButtonType GetInputMouseButton()
	{
		if(Input.GetMouseButton(0))
		{
			return MouseButtonType.Left;
		}
		else if(Input.GetMouseButton(1))
		{
			return MouseButtonType.Right;
		}
		else if(Input.GetMouseButton(2))
		{
			return MouseButtonType.Middle;
		}
		else
		{
			return MouseButtonType.None;
		}
	}
}