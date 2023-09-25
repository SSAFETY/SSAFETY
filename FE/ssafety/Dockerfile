FROM node:16-alpine as builder

WORKDIR /app

COPY . /app

RUN npm install
RUN npm run build

FROM nginx:latest

COPY --from=builder /app/build /usr/share/nginx/html

EXPOSE 80