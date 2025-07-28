from fastapi import FastAPI, File, UploadFile
from fastapi.responses import JSONResponse
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import shutil, os, re
from paddleocr import PaddleOCR

app =FastAPI()
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
ocr_model = PaddleOCR(use_angle_cls=True,
                      lang='korean',
                      text_detection_model_name="PP-OCRv5_mobile_det",
                      text_recognition_model_name="korean_PP-OCRv5_mobile_rec",
                      device="gpu"
                      )

origins = [
    "http://localhost",
    "http://localhost:8000",
    "http://127.0.0.1:8000",
    "http://0.0.0.0:8000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
UPLOAD_DIR = "uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)
static_path = os.path.join(BASE_DIR, "static", "ocr_test.html")

@app.get("/")
async def read_root():
    return FileResponse(static_path)


@app.post("/ocr")
async def ocr_image(file: UploadFile = File(...)):
    
    file_path = os.path.join(UPLOAD_DIR, file.filename)
    with open(file_path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)
    
    result = run_ocr(file_path)

    rec_texts = result[0].get("rec_texts", [])
    info = extract_info_from_result(rec_texts)
    return JSONResponse(content={"result": info})

def extract_info_from_result(rec_texts):
    phone = None
    order = None
    
    for text in rec_texts:
        if not phone:
            phone = extract_phone_number(text)
        if not order:
            order = extract_order_number(text)
    
    return {
        "result" : {
            "code" : order,
            "tel" : phone
        }
    }


def extract_phone_number(text):
    patterns = [
        r'0\d{1,2}[-\s]?\d{3,4}[-\s]?\d{4,5}',  
    ]
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return match.group()
    return None

def extract_order_number(text):
    pattern = r'주\s*문\s*번\s*호[:\s\-]*([A-Z0-9\-]+)'
    match = re.search(pattern, text, re.IGNORECASE)
    if match:
        return match.group(1) 
    return None


def run_ocr(image_path: str):
    result = ocr_model.predict(image_path)
    return result