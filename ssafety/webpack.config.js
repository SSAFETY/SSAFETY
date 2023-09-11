const HTMLWeebPackPlugin = require("html-webpack-plugin")
const path = require("path")

module.exports = {
  entry: path.resolve(__dirname, "./src/index.js"),
  module: {
    rules: [      {
      test: /\.(js|jsx)$/,
      exclude: /node_modules/,
      use: {
        loader: 'babel-loader'
      }
    }]
  },
  resolve: {
    extensions: ['.jsx', '.js'],
  },
  plugins: [
    new HTMLWeebPackPlugin({
      template: path.resolve(__dirname, "./src/index.html")
    })
  ],
  devServer: {
    compress: true,
    port: 3000,
    historyApiFallback: true
  }
}