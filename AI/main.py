from fastapi import FastAPI, HTTPException
import httpx

app = FastAPI()

PYTHON_URL = 'https://j11b209.p.ssafy.io/pyapi'
SPRING_URL = 'https://j11b209.p.ssafy.io/api'


async def send_coordinate(x, y, camera_id):
    async with httpx.AsyncClient() as client:
        try:
            response = await client.post(
                f"{PYTHON_URL}/transform_point/",
                data={
                    "x": x,
                    "y": y,
                    "camera_id": camera_id
                },
                timeout=10.0  # 요청 타임아웃 설정
            )
            response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
            
            
        except httpx.HTTPStatusError as e:
            return {"status": "error", "message": str(e)}

    try:
        response_data = response.json()
        if 'x_floor' in response_data and 'y_floor' in response_data:
            return {
                "status": "success",
                "x_floor": response_data["x_floor"],
                "y_floor": response_data["y_floor"]
            }
        else:
            return {"status": "error", "response": response_data}
    except ValueError:
        print("JSON 파싱 오류:", response.content)
        return {"status": "error", "response": response.content}
    

async def authentication():
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(f"{SPRING_URL}/connect", json={
                "AccessToken": "1234"  
            })
            response.raise_for_status() 
            return response.json() 
    except httpx.HTTPStatusError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        return {"status": "error", "message": str(e)}


async def disconnect(camera_id):
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(f"{SPRING_URL}/disconnect/{camera_id}", json={
                "AccessToken": "1234"  
            })
            response.raise_for_status() 
            return response.json() 
    except httpx.HTTPStatusError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        return {"status": "error", "message": str(e)}