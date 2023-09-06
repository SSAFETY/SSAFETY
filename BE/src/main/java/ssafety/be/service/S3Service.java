package ssafety.be.service;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.AmazonS3Exception;
import com.amazonaws.services.s3.model.ObjectMetadata;
import java.io.IOException;
import java.util.UUID;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

@Service
@RequiredArgsConstructor
public class S3Service {
    private final AmazonS3 amazonS3;
    private static final String bucketName = "a102";

    /**
     * 파일을 Amazon S3에 업로드하는 메서드입니다.
     *
     * @param file 업로드할 파일
     * @return String 업로드된 파일의 URL
     * @throws RuntimeException S3 업로드 중 발생한 오류 또는 파일 읽기 오류
     */
    public String uploadFile(MultipartFile file) {
        String originalFileName = file.getOriginalFilename();
        String extension = "";

        // 파일 이름에서 확장자 추출
        int dotIndex = originalFileName.lastIndexOf(".");
        if (dotIndex > 0 && dotIndex < originalFileName.length() - 1) {
            extension = originalFileName.substring(dotIndex + 1);
        } else {
            throw new RuntimeException("Invalid file name: " + originalFileName);
        }

        // 확장자별로 Content-Type 결정
        String contentType;
        switch (extension.toLowerCase()) {
            case "jpeg":
            case "jpg":
                contentType = "image/jpeg";
                break;
            case "png":
                contentType = "image/png";
                break;
            case "mp4":
                contentType = "video/mp4";
                break;
            default:
                throw new RuntimeException("올바르지 않은 확장자입니다.");
        }

        String fileName = System.currentTimeMillis() + "_" + UUID.randomUUID() + "." + extension;
        try {
            ObjectMetadata metadata = new ObjectMetadata();
            metadata.setContentLength(file.getSize());
            metadata.setContentType(contentType);

            amazonS3.putObject(bucketName, fileName, file.getInputStream(), metadata);
            return amazonS3.getUrl(bucketName, fileName).toString();
        } catch (AmazonS3Exception e) {
            throw new RuntimeException("Error while uploading to S3: " + e.getMessage());
        } catch (IOException e) {
            throw new RuntimeException("Error reading the file: " + e.getMessage());
        }
    }
}
