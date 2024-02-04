# S3 Reference

```bash
# List all s3 buckets
aws s3 ls --recursive

# View profiles (config)
cd ~/.aws
cat config

# Change profile
export AWS_PROFILE=profile_name

# Login
aws sso login

# Copy s3 file/folder to local drive
aws s3 sync s3://bucket_name/path/to/file /path/to/local/folder/
# or
aws s3 cp --recursive s3://bucket_name/path/to/file /path/to/local/folder/
```