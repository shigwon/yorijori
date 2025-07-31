FROM python:3.11

WORKDIR /paddleOCR

RUN apt-get update && apt-get install -y libgl1 libglib2.0-0

COPY ./requirements.txt /paddleOCR/requirements.txt

RUN pip install --upgrade pip && \
    python -m pip install paddlepaddle-gpu==3.1.0 -i https://www.paddlepaddle.org.cn/packages/stable/cu126/ && \
    pip install -r requirements.txt 

COPY . .

EXPOSE 8000

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]