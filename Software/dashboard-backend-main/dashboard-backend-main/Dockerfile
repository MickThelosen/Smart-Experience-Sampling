FROM python:3.11.4-alpine3.18

WORKDIR /backend_app
COPY . /backend_app

RUN pip install -r requirements.txt

EXPOSE 5000

CMD python ./main.py