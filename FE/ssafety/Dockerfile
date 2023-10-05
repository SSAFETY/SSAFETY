FROM node:18-alpine as builder

WORKDIR /app

COPY . /app

RUN npm install
RUN npm run build

FROM nginx:latest

COPY --from=builder /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80
